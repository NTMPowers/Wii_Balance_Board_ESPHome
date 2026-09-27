#include "wii.h"
#include "log.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "bluetooth.h"

#include <algorithm>
#include <bitset>
#include <array>
#include <cstring>
#include "utils.h"

static const char *TAG = "wii";

// Reject a re-page from the board for this long after we disconnect, restarted by any
// further attempt inside the window.
static constexpr uint64_t BOARD_RECONNECT_COOLDOWN_MS = 10000;
// How long sync() leaves an unpaired board able to connect and pair.
static constexpr uint64_t PAIRING_WINDOW_MS = 60000;
// Throttle for the refusal logged when an unpaired board keeps paging.
static constexpr uint64_t UNPAIRED_REFUSAL_LOG_INTERVAL_MS = 30000;

namespace esphome::wii_balance_board::detail {

class Wii::BalanceBoard {
  Bluetooth *bt;
  int queryState;
  uint16_t handle;
  std::array<uint16_t, 12> calibration;

  uint8_t referenceTemperature{0};
  bool lastAButtonPressed{false};

  // The board holds its 0/17/34 kg calibration points and a reference temperature in
  // EEPROM. COLLECTING averages the raw sensors over TARE_WINDOW_MS, the result becomes
  // the new 0 kg points, and the temperature read during that window becomes the new
  // reference temperature.
  enum class TareStage : uint8_t { IDLE, COLLECTING, WRITE_ZERO, WRITE_TEMP, VERIFY_ZERO, VERIFY_TEMP };

  static constexpr uint32_t TARE_WINDOW_MS = 2000;
  static constexpr uint8_t TARE_MAX_ATTEMPTS = 3;
  // Per-sensor count range over the window that still counts as an undisturbed board.
  static constexpr uint16_t TARE_MAX_SPREAD = 200;
  // Total weight above which the board is carrying something and must not be zeroed.
  static constexpr float TARE_MAX_LOAD_KG = 5.0f;

  TareStage tareStage{TareStage::IDLE};
  uint32_t tareWindowStartMs{0};
  uint8_t tareAttempt{0};
  uint32_t tareSum[4]{};
  uint16_t tareMin[4]{};
  uint16_t tareMax[4]{};
  uint32_t tareSamples{0};
  uint8_t tareTemperature{0};
  uint8_t calRaw[24]{};     // 0xA40024..0xA4003B exactly as read
  uint8_t crcHeader[2]{};   // 0xA40020, 0xA40021
  uint8_t crcStored[4]{};   // 0xA4003C..0xA4003F
  uint8_t refTempByte61{0x01};
  uint8_t pendingZero[8]{}; // new 0 kg points, big endian, ready to write
  std::function<void(uint16_t, bool)> onTared_;

  void restartTareWindow_() {
    tareWindowStartMs = millis();
    tareSamples = 0;
    std::memset(tareSum, 0, sizeof(tareSum));
  }

  void failTare_(const char *reason) {
    ESP_LOGE(TAG, "Board zeroing failed: %s", reason);
    tareStage = TareStage::IDLE;
    queryState = 0;
    if (onTared_) {
      onTared_(handle, false);
    }
  }

  void tareAccumulate_(const uint8_t *mem) {
    const uint16_t values[4] = {
        static_cast<uint16_t>(mem[0] * 256 + mem[1]),
        static_cast<uint16_t>(mem[2] * 256 + mem[3]),
        static_cast<uint16_t>(mem[4] * 256 + mem[5]),
        static_cast<uint16_t>(mem[6] * 256 + mem[7]),
    };
    tareTemperature = mem[8];
    for (uint8_t i = 0; i < 4; ++i) {
      if (tareSamples == 0) {
        tareMin[i] = values[i];
        tareMax[i] = values[i];
      } else {
        tareMin[i] = std::min(tareMin[i], values[i]);
        tareMax[i] = std::max(tareMax[i], values[i]);
      }
      tareSum[i] += values[i];
    }
    tareSamples++;
  }

  // Returns true when a usable empty-board average is ready to be written.
  bool tareFinalize_() {
    if (tareSamples == 0) {
      return false;
    }
    uint16_t average[4];
    for (uint8_t i = 0; i < 4; ++i) {
      average[i] = static_cast<uint16_t>(tareSum[i] / tareSamples);
    }
    for (uint8_t i = 0; i < 4; ++i) {
      if (static_cast<uint32_t>(tareMax[i] - tareMin[i]) > TARE_MAX_SPREAD) {
        ESP_LOGD(TAG, "Board was disturbed while zeroing (sensor %u spread %u counts), averaging again",
                 static_cast<unsigned>(i), static_cast<unsigned>(tareMax[i] - tareMin[i]));
        return retryTare_();
      }
    }

    float total = 0.0f;
    for (uint8_t i = 0; i < 4; ++i) {
      total += interpolate(i, average) / 1000.0f;
    }
    if (total > TARE_MAX_LOAD_KG) {
      ESP_LOGD(TAG, "Board reads %.2f kg before zeroing, averaging again", static_cast<double>(total));
      return retryTare_();
    }

    for (uint8_t i = 0; i < 4; ++i) {
      pendingZero[i * 2] = static_cast<uint8_t>(average[i] >> 8);
      pendingZero[i * 2 + 1] = static_cast<uint8_t>(average[i] & 0xFF);
    }
    ESP_LOGD(TAG,
             "Averaged %lu empty samples: TR=%u BR=%u TL=%u BL=%u, temperature=%u, factory read %.2f kg",
             static_cast<unsigned long>(tareSamples), static_cast<unsigned>(average[0]),
             static_cast<unsigned>(average[1]), static_cast<unsigned>(average[2]), static_cast<unsigned>(average[3]),
             static_cast<unsigned>(tareTemperature), static_cast<double>(total));
    return true;
  }

  bool retryTare_() {
    if (++tareAttempt >= TARE_MAX_ATTEMPTS) {
      failTare_("board never settled empty");
      return false;
    }
    restartTareWindow_();
    return false;
  }

 public:
  BalanceBoard(Bluetooth *bt, uint16_t handle, std::function<void(uint16_t, bool)> onTared)
      : bt(bt), queryState(0), handle(handle), onTared_(std::move(onTared)) {}

  // The window is normally closed by the arrival of a report. A board that never
  // streams one would otherwise be reported as a session timeout.
  void tick() {
    if (tareStage != TareStage::COLLECTING) {
      return;
    }
    const uint32_t elapsed = static_cast<uint32_t>(millis() - tareWindowStartMs);
    if (elapsed > TARE_WINDOW_MS && tareSamples == 0) {
      failTare_("board sent no weight reports to average");
    }
  }

  void setLeds(Bluetooth *bt, uint16_t handle, const std::bitset<4> &bits) {
    uint8_t ledData[] = {
        0xA2,
        0x11,
        static_cast<uint8_t>(bits.to_ulong() << 4),
    };

    bt->l2send_data(handle, 0x0013, ledData, 3);
  }

  void set_reporting_mode(uint16_t handle, uint8_t reportingMode, bool continuous) {
    uint8_t data[] = {0xA2, 0x12, (uint8_t) (continuous ? 0x04 : 0x00), reportingMode};
    bt->l2send_data(handle, 0x0013, data, 4);
  }

  void write_memory(uint16_t handle, uint8_t addressSpace, uint32_t offset, std::initializer_list<uint8_t> memData) {
    uint8_t data[] = {0xA2,
                      0x16,
                      addressSpace,
                      (uint8_t) ((offset >> 16) & 0xFF),
                      (uint8_t) ((offset >> 8) & 0xFF),
                      (uint8_t) ((offset) &0xFF),
                      (uint8_t) (memData.size()),
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00};
    std::copy(memData.begin(), memData.end(), data + 7);

    bt->l2send_data(handle, 0x0013, data, 23);
  }

  void read_memory(uint16_t handle, uint8_t addressSpace, uint32_t offset, uint16_t size) {
    uint8_t data[] = {0xA2,
                      0x17,
                      addressSpace,
                      (uint8_t) ((offset >> 16) & 0xFF),
                      (uint8_t) ((offset >> 8) & 0xFF),
                      (uint8_t) ((offset) &0xFF),
                      (uint8_t) ((size >> 8) & 0xFF),
                      (uint8_t) ((size) &0xFF)};
    bt->l2send_data(handle, 0x0013, data, 8);

    uint16_t data_len = 8;
  }

  int32_t interpolate(uint8_t pos, uint16_t *values) {
    uint16_t *cal = calibration.data();
    float weight = 0;

    if (values[pos] < cal[pos]) {  // 0kg
      weight = 0;
    } else if (values[pos] < cal[pos + 4]) {  // 17kg
      weight = 17 * (float) (values[pos] - cal[pos]) / (float) (cal[pos + 4] - cal[pos]);
    } else {  // 34kg
      weight = 17 + 17 * (float) (values[pos] - cal[pos + 4]) / (float) (cal[pos + 8] - cal[pos + 4]);
    }

    return weight * 1000;
  }

  void readCalibrationData(uint8_t *data, size_t len) {
    if (len < 2) {
      ESP_LOGW(TAG, "Ignoring short calibration report (%u bytes)", static_cast<unsigned>(len));
      return;
    }

    switch (queryState) {
      case 0:
        if (data[1] == 0x20 && len >= 5 && (data[4] & 0x02)) {
          write_memory(handle, 0x04, 0xA400F0, {0x55});  // Disable encryption 1
          queryState = 1;
        }
        break;
      case 1:
        if (data[1] == 0x22 && len >= 6 && data[4] == 0x16) {
          if (data[5] == 0x00) {
            write_memory(handle, 0x04, 0xA400FB, {0x00});  // Disable encryption 2
            queryState = 2;
          } else {
            ESP_LOGW(TAG, "Calibration write 1 failed, restarting calibration");
            queryState = 0;
          }
        }
        break;
      case 2:
        if (data[1] == 0x22 && len >= 6 && data[4] == 0x16) {
          if (data[5] == 0x00) {
            read_memory(handle, 0x04, 0xA400FA, 6);
            queryState = 3;
          } else {
            ESP_LOGW(TAG, "Calibration write 2 failed, restarting calibration");
            queryState = 0;
          }
        }
        break;
      case 3:
        if (data[1] == 0x21 && len >= 13) {
          // Wii Balance Board
          if (memcmp(data + 5, (const uint8_t[]){0x00, 0xFA, 0x00, 0x00, 0xA4, 0x20, 0x04, 0x02}, 8) == 0) {
            read_memory(handle, 0x04, 0xA40024, 16);  // read calibration 0 kg and 17kg
            queryState = 4;
          } else {
            ESP_LOGW(TAG, "Unexpected calibration identity response, restarting calibration");
            queryState = 0;
          }
        }
        break;
      case 4:
        if (data[1] != 0x21 || len < 23) {
          break;
        }
        ESP_LOGD(TAG, "Calibration data 0kg and 17kg");
        {
        uint8_t *mem = data + 7;

        std::memcpy(calRaw, mem, 16);

        calibration[0] = mem[0] * 256 + mem[1];  // Top Right 0kg
        calibration[1] = mem[2] * 256 + mem[3];  // Bottom Right 0kg
        calibration[2] = mem[4] * 256 + mem[5];  // Top Left 0kg
        calibration[3] = mem[6] * 256 + mem[7];  // Bottom Left 0kg

        calibration[4] = mem[8] * 256 + mem[9];    // Top Right 17kg
        calibration[5] = mem[10] * 256 + mem[11];  // Bottom Right 17kg
        calibration[6] = mem[12] * 256 + mem[13];  // Top Left 17kg
        calibration[7] = mem[14] * 256 + mem[15];  // Bottom Left 17kg
        }
        read_memory(handle, 0x04, 0xA40034, 8);  // read calibration 34kg

        queryState = 5;
        break;
      case 5:
        if (data[1] != 0x21 || len < 15) {
          break;
        }
        ESP_LOGD(TAG, "Calibration data 34kg");
        {
          uint8_t *mem = data + 7;
          std::memcpy(calRaw + 16, mem, 8);
          calibration[8] = mem[0] * 256 + mem[1];   // Top Right 34kg
          calibration[9] = mem[2] * 256 + mem[3];   // Bottom Right 34kg
          calibration[10] = mem[4] * 256 + mem[5];  // Top Left 34kg
          calibration[11] = mem[6] * 256 + mem[7];  // Bottom Left 34kg
        }
        read_memory(handle, 0x04, 0xA40060, 2);  // read calibration reference temperature

        queryState = 6;
        break;
      case 6:
        if (data[1] != 0x21 || len < 9) {
          break;
        }
        {
        uint8_t *mem = data + 7;
        ESP_LOGD(TAG, "Calibration data reference temperature");
        referenceTemperature = mem[0];
        refTempByte61 = mem[1];
        if (referenceTemperature == 0) {
          ESP_LOGW(TAG, "Calibration returned zero reference temperature");
          queryState = 0;
          break;
        }
        for (size_t i = 0; i < 4; ++i) {
          if (calibration[i] >= calibration[i + 4] || calibration[i + 4] >= calibration[i + 8]) {
            ESP_LOGW(TAG, "Invalid calibration values for sensor %u", static_cast<unsigned>(i));
            referenceTemperature = 0;
            queryState = 0;
            return;
          }
        }
        ESP_LOGD(TAG, "Calibration complete, reference temperature=%u", referenceTemperature);
        // The two bytes the checksum covers from 0xA40020, then the checksum itself.
        read_memory(handle, 0x04, 0xA40020, 2);
        queryState = 7;
        break;
        }
      default:
        handleTareResponse_(data, len);
        break;
    }
  }

  // States 7 and up cover zeroing the board. Kept out of the switch above so the
  // factory read path stays readable.
  void handleTareResponse_(uint8_t *data, size_t len) {
    switch (queryState) {
      case 7:
        if (data[1] != 0x21 || len < 9) {
          return;
        }
        crcHeader[0] = data[7];
        crcHeader[1] = data[8];
        read_memory(handle, 0x04, 0xA4003C, 4);  // read the stored block checksum
        queryState = 8;
        return;
      case 8: {
        if (data[1] != 0x21 || len < 11) {
          return;
        }
        std::memcpy(crcStored, data + 7, 4);

        // Log the factory block so the original calibration can be restored by hand.
        ESP_LOGD(TAG,
                 "Factory block 0x20=%02X%02X 0kg=%02X%02X %02X%02X %02X%02X %02X%02X 17kg=%02X%02X %02X%02X %02X%02X "
                 "%02X%02X 34kg=%02X%02X %02X%02X %02X%02X %02X%02X 0x60=%02X%02X 0x3C=%02X%02X%02X%02X",
                 crcHeader[0], crcHeader[1], calRaw[0], calRaw[1], calRaw[2], calRaw[3], calRaw[4], calRaw[5],
                 calRaw[6], calRaw[7], calRaw[8], calRaw[9], calRaw[10], calRaw[11], calRaw[12], calRaw[13],
                 calRaw[14], calRaw[15], calRaw[16], calRaw[17], calRaw[18], calRaw[19], calRaw[20], calRaw[21],
                 calRaw[22], calRaw[23], referenceTemperature, refTempByte61, crcStored[0], crcStored[1], crcStored[2],
                 crcStored[3]);
        ESP_LOGD(TAG, "Factory 0kg TR=%u BR=%u TL=%u BL=%u, reference temperature=%u",
                 static_cast<unsigned>(calibration[0]), static_cast<unsigned>(calibration[1]),
                 static_cast<unsigned>(calibration[2]), static_cast<unsigned>(calibration[3]),
                 static_cast<unsigned>(referenceTemperature));

        // 0xA4003C is left as the factory set it. No CRC32 over this block
        // reproduces the stored value in either byte order, so there is no correct
        // value to write back.
        ESP_LOGD(TAG, "Leaving the block checksum %02X%02X%02X%02X at 0xA4003C as the factory set it",
                 crcStored[0], crcStored[1], crcStored[2], crcStored[3]);

        // The board only streams weight reports once it has been told which mode
        // to use, so this must precede the averaging window.
        set_reporting_mode(handle, 0x34, true);
        tareAttempt = 0;
        tareStage = TareStage::COLLECTING;
        restartTareWindow_();
        return;
      }
      case 9:  // 0 kg points written, now the reference temperature
      case 10:  // temperature written, now verify
        if (data[1] != 0x22 || len < 6) {
          return;
        }
        if (data[5] != 0x00) {
          failTare_("the board rejected a write");
          return;
        }
        if (queryState == 9) {
          write_memory(handle, 0x04, 0xA40060, {tareTemperature, refTempByte61});
          queryState = 10;
        } else {
          read_memory(handle, 0x04, 0xA40024, 16);  // verify the 0 kg points landed
          queryState = 11;
        }
        return;
      case 11: {
        if (data[1] != 0x21 || len < 23) {
          return;
        }
        if (std::memcmp(data + 7, pendingZero, 8) != 0) {
          failTare_("the 0 kg points did not read back correctly");
          return;
        }
        // Adopt the values the board actually holds, so weight maths uses what is
        // really in EEPROM rather than what we meant to write.
        std::memcpy(calRaw, data + 7, 16);
        for (uint8_t i = 0; i < 4; ++i) {
          calibration[i] = data[7 + i * 2] * 256 + data[8 + i * 2];
        }
        read_memory(handle, 0x04, 0xA40060, 2);
        queryState = 12;
        return;
      }
      case 12: {
        if (data[1] != 0x21 || len < 9) {
          return;
        }
        if (data[7] != tareTemperature) {
          failTare_("the reference temperature did not read back correctly");
          return;
        }
        // The board now reports weight against this temperature.
        referenceTemperature = tareTemperature;
        tareStage = TareStage::IDLE;
        queryState = 0;
        set_reporting_mode(handle, 0x34, false);
        ESP_LOGD(TAG, "Board zeroed: 0kg TR=%u BR=%u TL=%u BL=%u, reference temperature=%u",
                 static_cast<unsigned>(calibration[0]), static_cast<unsigned>(calibration[1]),
                 static_cast<unsigned>(calibration[2]), static_cast<unsigned>(calibration[3]),
                 static_cast<unsigned>(referenceTemperature));
        if (onTared_) {
          onTared_(handle, true);
        }
        return;
      }
      default:
        return;
    }
  }

  bool onData(BalanceBoardData *out, uint16_t handle, uint8_t *data, size_t len) {
    if (len < 2) {
      ESP_LOGD(TAG, "Ignoring short board report (%u bytes)", static_cast<unsigned>(len));
      return false;
    }
    if (data[0] == 0xA1) {
      if (len >= 4 && data[1] != 0x3D) {
        const bool aButtonPressed = (data[3] & 0x08) != 0;
        if (aButtonPressed && !lastAButtonPressed) {
          ESP_LOGD(TAG, "Balance Board A button pressed");
        }
        lastAButtonPressed = aButtonPressed;
      }
      // A non-zero reference temperature means we have read the calibration block
      if (data[1] == 0x34 && referenceTemperature != 0) {
        if (len < 15) {
          ESP_LOGD(TAG, "Ignoring short balance report (%u bytes)", static_cast<unsigned>(len));
          return false;
        }
        uint8_t *mem = data + 4;

        // While zeroing, reports feed the average instead of producing a measurement.
        if (tareStage == TareStage::COLLECTING) {
          tareAccumulate_(mem);
          if (static_cast<uint32_t>(millis() - tareWindowStartMs) >= TARE_WINDOW_MS && tareFinalize_()) {
            tareStage = TareStage::WRITE_ZERO;
            write_memory(handle, 0x04, 0xA40024, {pendingZero[0], pendingZero[1], pendingZero[2], pendingZero[3],
                                                   pendingZero[4], pendingZero[5], pendingZero[6], pendingZero[7]});
            queryState = 9;
          }          return false;
        }
        if (tareStage != TareStage::IDLE) {
          return false;  // ignore reports while the new zero is written and checked
        }

        uint16_t values[4] = {
            static_cast<uint16_t>(mem[0] * 256 + mem[1]),  // tr
            static_cast<uint16_t>(mem[2] * 256 + mem[3]),  // br
            static_cast<uint16_t>(mem[4] * 256 + mem[5]),  // tl
            static_cast<uint16_t>(mem[6] * 256 + mem[7]),  // bl
        };
        out->handle = handle;
        out->tr = interpolate(0, values);
        out->br = interpolate(1, values);
        out->tl = interpolate(2, values);
        out->bl = interpolate(3, values);
        out->temperature = mem[8];
        out->batteryLevel = mem[10];
        out->referenceTemperature = referenceTemperature;
        return true;
      } else {
        readCalibrationData(data, len);
      }
    }
    return false;
  }
};

Wii::Wii(Bluetooth *bt) : bluetooth(bt) {
  bt->onHCIConnectionRequest([this](Bluetooth *, const HCIConnectionRequest &result) {
    ESP_LOGD(TAG, "Received connection request at %lu ms from %s", static_cast<unsigned long>(millis()),
             formatHex((uint8_t *) &result.bdaddr, 6));
    if (result.classOfDevice == 0x042500) {
      uint64_t now = millis();
      if (now < rejectBoardUntil_) {
        // A board still standing on it re-pages repeatedly, so only the first
        // refusal in a window is worth an INFO line.
        if (cooldownLogged_) {
          ESP_LOGD(TAG, "Rejecting board reconnect during post-disconnect cooldown (%lu ms left)",
                   static_cast<unsigned long>(rejectBoardUntil_ - now));
        } else {
          cooldownLogged_ = true;
          ESP_LOGI(TAG, "Ignoring a board reconnect during the post-disconnect cooldown");
        }
        rejectBoardUntil_ = now + BOARD_RECONNECT_COOLDOWN_MS;  // restart the cooldown window
        return false;
      }
      // A board with no stored key would fall through to PIN entry and pair
      // itself, so only let one in during an explicit pairing window.
      if (!pairingAllowed_() && !bluetooth->hasLinkKey(result.bdaddr)) {
        if (now - lastUnpairedRefusalLogMs_ >= UNPAIRED_REFUSAL_LOG_INTERVAL_MS) {
          lastUnpairedRefusalLogMs_ = now;
          ESP_LOGI(TAG, "Refused unpaired board %012llX", static_cast<unsigned long long>(result.bdaddr));
        } else {
          ESP_LOGD(TAG, "Refused a connection from unpaired board %012llX",
                   static_cast<unsigned long long>(result.bdaddr));
        }
        return false;
      }
      ESP_LOGD(TAG, "Accepting board connection from paired device");
      return true;  // Accept incoming connections from balance board
    }
    return false;  // Reject all other incoming connections
  });

  bt->onHCIEvent([this](Bluetooth *bt, const HCIEvent &event) {
    std::visit(overloaded{
                   [this](const HCIInquiryStarted &) { this->eventListener(ScanStarted{}); },
                   [this](const HCIInquiryComplete &) { this->eventListener(ScanStopped{}); },
                   [bt](const HCIInquiryResult &result) {
                     if (result.classOfDevice == 0x042500) {
                       bt->requestRemoteName(result);
                     }
                   },
                    [bt](const HCIRemoteName &result) {
                      ESP_LOGD(TAG, "Found %s %s", result.remoteName.data(), formatHex((uint8_t *) &result.inquiry.bdaddr, 6));
                      if (result.remoteName == "Nintendo RVL-WBC-01") {
                        bt->connect(result.inquiry);
                      }
                    },
                     [this](const HCIRoleChanged &result) {
                        // The board only initiates L2CAP while it is the link slave, so a
                        // settled slave role strands the session. A switch requested here
                        // can collide with the board's own LMP transaction and fail, so
                        // record it and force master once encryption completes.
                        if (result.status != 0x00) {
                         needsRoleSwitch.emplace(result.bdaddr);
                         ESP_LOGD(TAG, "Role switch failed status=0x%02X for %s, will force master once encrypted",
                                  result.status, formatHex((uint8_t *) &result.bdaddr, 6));
                       } else if (result.newRole == 0x00) {
                         needsRoleSwitch.erase(result.bdaddr);
                         ESP_LOGD(TAG, "Link role settled: host is MASTER for %s, board will open L2CAP",
                                  formatHex((uint8_t *) &result.bdaddr, 6));
                       } else {
                         needsRoleSwitch.erase(result.bdaddr);
                         ESP_LOGD(TAG, "Link role settled: host is SLAVE for %s, board will not open L2CAP",
                                  formatHex((uint8_t *) &result.bdaddr, 6));
                       }
                     },
                      [this](const HCIConnectionFailed &result) {
                        pendingReconnect.reset();
                        reconnecting = false;
                        if (result.reason == 0x0F) {
                      ESP_LOGD(TAG, "Connection request rejected for %s",
                               formatHex((uint8_t *) &result.bdaddr, 6));
                        } else {
                      ESP_LOGW(TAG, "Failed to connect board %s reason=0x%02X",
                               formatHex((uint8_t *) &result.bdaddr, 6), result.reason);
                        }
                      },
                        [this](const HCIConnectionEstablished &result) {
                          ESP_LOGD(TAG, "Board link established handle=%u", result.handle);

                         pendingReconnect.reset();
                         reconnecting = false;
                         handleToBdaddr[result.handle] = result.bdaddr;

                         // The board refuses host-initiated L2CAP when it is connection
                         // master, as on a reconnect where it paged us.
                         if (!result.accepted) {
                           initiatorHandles.emplace(result.handle);
                         }

                         // Newer boards power off if PSM 0x0013 is opened pre-auth, so L2CAP
                         // is deferred to HCIEncryptionChange.
                         bluetooth->auth(result.handle);
                      },
                    [bt](const HCILinkKeyRequest &result) {
                      ESP_LOGD(TAG, "Negative link reply");
                      bt->negativeReply(result.bdaddr);
                    },
                    [this](const HCIPINRequest &result) {
                      // Answering a PIN request is what pairs a board, so it is
                      // only done for a board the user has asked to pair.
                      if (!pairingAllowed_()) {
                        ESP_LOGW(TAG, "Refused a PIN request from unpaired board %012llX",
                                 static_cast<unsigned long long>(result.bdaddr));
                        return;
                      }
                      uint8_t pin_data[6];
                      auto mac = bluetooth->macAddress();
                      for (size_t i = 0; i < 6; ++i) {
                        pin_data[i] = mac[5 - i];
                      }
                      ESP_LOGD(TAG, "Sending pin reply");
                      bluetooth->sendPinReply(result.bdaddr, pin_data, 6);
                    },
                           [this](const HCIDisconnected &result) {
                             ESP_LOGD(TAG, "Disconnected handle=%u reason=0x%02X", result.handle, result.reason);
                             pendingPSM13.erase(result.handle);
                             pendingEncryption.erase(result.handle);
                             initiatorHandles.erase(result.handle);
                             auto gone = handleToBdaddr.find(result.handle);
                             if (gone != handleToBdaddr.end()) {
                               needsRoleSwitch.erase(gone->second);
                             }
                             handleToBdaddr.erase(result.handle);
                             // HCIDisconnected is the only authoritative end-of-link
                             // signal, so report the board gone from here alone.
                             handleBoardGone_(result.handle);
                          },
                           [this](const HCIAuthComplete &result) {
                            if (result.status == 0x00) {
                              ESP_LOGD(TAG, "Auth complete for handle=0x%04X, enabling encryption", result.handle);
                              pendingEncryption.emplace(result.handle);
                              bluetooth->setEncryption(result.handle);
                            } else {
                              pendingEncryption.erase(result.handle);
                              ESP_LOGW(TAG, "Auth failed for handle=0x%04X status=0x%02X", result.handle, result.status);
                            }
                            },
                           [this](const HCIEncryptionChange &result) {
                            if (result.status == 0x00 && pendingEncryption.erase(result.handle) > 0) {
                              if (initiatorHandles.erase(result.handle) > 0) {
                                ESP_LOGD(TAG, "Encryption enabled for handle=0x%04X, opening PSM 0x0011", result.handle);
                                pendingPSM13.emplace(result.handle);
                                bluetooth->l2cap_connect(result.handle, 0x0011, 0x40);
                              } else {
                                // Encryption is late enough that the board's own switch
                                // attempt has finished, so this retry cannot collide.
                                auto it = handleToBdaddr.find(result.handle);
                                uint64_t bdaddr = it != handleToBdaddr.end() ? it->second : 0;
                                if (bdaddr != 0 && needsRoleSwitch.erase(bdaddr) > 0) {
                                  ESP_LOGD(TAG,
                                           "Encryption enabled for handle=0x%04X, forcing master for %s after role switch collision",
                                           result.handle, formatHex((uint8_t *) &bdaddr, 6));
                                  bluetooth->switch_role(bdaddr);
                                } else {
                                  ESP_LOGD(TAG, "Encryption enabled for handle=0x%04X, waiting for board to open L2CAP",
                                           result.handle);
                                }
                              }
                            } else if (result.status != 0x00) {
                              pendingEncryption.erase(result.handle);
                              ESP_LOGW(TAG, "Encryption failed for handle=0x%04X status=0x%02X", result.handle, result.status);
                            }
                            },
                 },
                 event);
  });

  bt->onACLConnectionRequest([](Bluetooth *, const ACLConnectionRequest &req) {
    ESP_LOGD(TAG, "Received ACL connection request from %u, psm %02X", req.handle, req.psm);
    return (req.psm == 0x0011 || req.psm == 0x0013);
  });

  bt->onACLEvent([this](Bluetooth *bt, const ACLEvent &event) {
    std::visit(overloaded{
                   [this](const ACLConnectionFailed &) {},
                       [this](const ACLDisconnected &info) { this->onACLChannelClosedUnexpectedly_(info.handle, info.psm); },
                     [this, bt](const ACLConnectionEstablished &conn) {
                       if (conn.psm == 0x0011 && pendingPSM13.erase(conn.handle) > 0) {
                         ESP_LOGD(TAG, "PSM 0x0011 established for handle=%u, opening PSM 0x0013", conn.handle);
                         bt->l2cap_connect(conn.handle, 0x0013, 0x40);
                        } else if (conn.psm == 0x0013) {
                          pendingPSM13.erase(conn.handle);
                          ESP_LOGD(TAG, "PSM 0x0013 established for handle=%u, board ready", conn.handle);
                          connectedBoards.emplace(
                              conn.handle, std::make_unique<BalanceBoard>(bluetooth, conn.handle, [this](uint16_t h, bool ok) {
                                this->eventListener(BalanceBoardTared{.handle = h, .ok = ok});
                              }));
                          connectedBoards[conn.handle]->setLeds(bluetooth, conn.handle, std::bitset<4>(0b0001));
                          this->eventListener(BalanceBoardConnected{
                              .handle = conn.handle,
                              .bdaddr = handleToBdaddr[conn.handle],
                          });
                        }
                     },
                   [this](const ACLData &data) {
                     auto board = connectedBoards.find(data.handle);
                     if (board == connectedBoards.end()) {
                       ESP_LOGD(TAG, "Ignoring ACL data for inactive handle=%u channel=0x%04X", data.handle,
                                data.channelId);
                       return;
                     }
                     BalanceBoardData out;
                     if (board->second->onData(&out, data.handle, data.data, data.len)) {
                       this->eventListener(out);
                     }
                   },
               },
               event);
  });
}

Wii::~Wii() {}

void Wii::sync(bool enable) {
  if (enable) {
    pairingAllowedUntil_ = millis() + PAIRING_WINDOW_MS;
  }
  bluetooth->scan(enable);
}

void Wii::step() {
  if (pendingReconnect.has_value() && !reconnecting) {
    reconnecting = true;
    auto bdaddr = pendingReconnect.value();
    pendingReconnect.reset();
    ESP_LOGD(TAG, "Reconnecting to board %s", formatHex((uint8_t *) &bdaddr, 6));
    bluetooth->connect(bdaddr);
  }
  bluetooth->process();
  for (auto &entry : connectedBoards) {
    entry.second->tick();
  }
}

void Wii::onEvent(std::function<void(const WiiEvent &)> eventListener) {
  this->eventListener = std::move(eventListener);
}

void Wii::disconnect(uint16_t handle) {
  // A single unconditional HCI disconnect. The controller tears down the L2CAP
  // channels itself; waiting on their own close notifications would leave the
  // board able to treat a half-torn-down link as merely dropped.
  ESP_LOGD(TAG, "Disconnecting board handle=%u", handle);
  bluetooth->disconnect(handle);
}

bool Wii::remove_link_key(uint64_t bdaddr) { return bluetooth->removeLinkKey(bdaddr); }

int Wii::remove_all_link_keys() { return bluetooth->removeAllLinkKeys(); }

void Wii::handleBoardGone_(uint16_t handle) {
  if (connectedBoards.erase(handle) > 0) {
    rejectBoardUntil_ = millis() + BOARD_RECONNECT_COOLDOWN_MS;
    cooldownLogged_ = false;
    this->eventListener(BalanceBoardDisconnected{.handle = handle});
  }
}

void Wii::onACLChannelClosedUnexpectedly_(uint16_t handle, uint16_t psm) {
  // The board never asks to disconnect, so a channel closing here means the link
  // is in an indeterminate state. Tear it down fully.
  ESP_LOGW(TAG, "L2CAP channel psm=0x%04X closed unexpectedly for handle=%u; forcing full disconnect", psm, handle);
  bluetooth->disconnect(handle);
}

}  // namespace esphome::wii_balance_board::detail
