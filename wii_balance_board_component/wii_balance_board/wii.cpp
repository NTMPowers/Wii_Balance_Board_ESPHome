#include "wii.h"
#include "log.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "bluetooth.h"

#include <bitset>
#include <array>
#include <cstring>
#include "utils.h"

static const char *TAG = "wii";

// After we disconnect the board, briefly reject any immediate re-page from it. The board
// itself re-pages within a few seconds of a clean disconnect while a user is still standing
// on it (observed on hardware); rejecting a paired device is safe since it is always still
// trusted and can page again once the cooldown lapses. Any re-page attempt seen during the
// cooldown restarts the window, matching the requested behavior of blocking reconnects for
// 10s after each attempt, not just after the original disconnect.
static constexpr uint64_t BOARD_RECONNECT_COOLDOWN_MS = 10000;

namespace esphome::wii_balance_board::detail {

class Wii::BalanceBoard {
  Bluetooth *bt;
  int queryState;
  uint16_t handle;
  std::array<uint16_t, 12> calibration;

  uint8_t referenceTemperature{0};
  bool lastAButtonPressed{false};

 public:
  BalanceBoard(Bluetooth *bt, uint16_t handle) : bt(bt), handle(handle), queryState(0) {}

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
        set_reporting_mode(handle, 0x34, false);
        queryState = 0;
        break;
        }
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
      // A non-zero reference temperature means we have calibrated
      if (data[1] == 0x34 && referenceTemperature != 0) {
        if (len < 15) {
          ESP_LOGD(TAG, "Ignoring short balance report (%u bytes)", static_cast<unsigned>(len));
          return false;
        }
        uint8_t *mem = data + 4;

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
        ESP_LOGD(TAG, "Rejecting board reconnect during post-disconnect cooldown (%lu ms left)",
                 static_cast<unsigned long>(rejectBoardUntil_ - now));
        rejectBoardUntil_ = now + BOARD_RECONNECT_COOLDOWN_MS;  // restart the cooldown window
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
                       // The settled link role decides what happens next: when the board ends up
                       // master it never opens L2CAP itself and we only wait for it (see
                       // HCIEncryptionChange), so losing the accept-time role race strands the
                       // session until the board drops the link.
                       if (result.status == 0x00 && result.newRole == 0x00) {
                         ESP_LOGD(TAG, "Link role settled: host is MASTER for %s, board will open L2CAP",
                                  formatHex((uint8_t *) &result.bdaddr, 6));
                       } else if (result.status == 0x00) {
                         ESP_LOGD(TAG, "Link role settled: host is SLAVE for %s, board will not open L2CAP",
                                  formatHex((uint8_t *) &result.bdaddr, 6));
                       } else {
                         ESP_LOGD(TAG, "Role switch failed status=0x%02X for %s, host stays SLAVE, board will not open L2CAP",
                                  result.status, formatHex((uint8_t *) &result.bdaddr, 6));
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

                         // The board refuses host-initiated L2CAP when it is the connection master
                         // (i.e. it paged us, as on reconnect); only self-initiate L2CAP when we
                         // were the one who paged the board (master), matching pairing behavior.
                         if (!result.accepted) {
                           initiatorHandles.emplace(result.handle);
                         }

                         // Do not open L2CAP before auth/encryption: newer boards power off if the
                         // data pipe (PSM 0x0013) is opened pre-auth. L2CAP is opened later, from
                         // HCIEncryptionChange, once the link is authenticated and encrypted.
                         bluetooth->auth(result.handle);
                      },
                    [bt](const HCILinkKeyRequest &result) {
                      ESP_LOGD(TAG, "Negative link reply");
                      bt->negativeReply(result.bdaddr);
                    },
                    [bt](const HCIPINRequest &result) {
                      uint8_t pin_data[6];
                      auto mac = bt->macAddress();
                      for (size_t i = 0; i < 6; ++i) {
                        pin_data[i] = mac[5 - i];
                      }
                      ESP_LOGD(TAG, "Sending pin reply");
                      bt->sendPinReply(result.bdaddr, pin_data, 6);
                    },
                           [this](const HCIDisconnected &result) {
                             ESP_LOGD(TAG, "Disconnected handle=%u reason=0x%02X", result.handle, result.reason);
                             pendingPSM13.erase(result.handle);
                             pendingEncryption.erase(result.handle);
                             initiatorHandles.erase(result.handle);
                             handleToBdaddr.erase(result.handle);
                             // The Bluetooth link is now definitively gone (this is the only ground-truth
                             // signal for that, matching how a Linux host determines "disconnected"). Only
                             // now do we tell the app layer the board is disconnected, never earlier from
                             // an intermediate L2CAP-level event, so our belief always matches reality.
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
                                ESP_LOGD(TAG, "Encryption enabled for handle=0x%04X, waiting for board to open L2CAP", result.handle);
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
                         connectedBoards.emplace(conn.handle, std::make_unique<BalanceBoard>(bluetooth, conn.handle));
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

void Wii::sync(bool enable) { bluetooth->scan(enable); }

void Wii::step() {
  if (pendingReconnect.has_value() && !reconnecting) {
    reconnecting = true;
    auto bdaddr = pendingReconnect.value();
    pendingReconnect.reset();
    ESP_LOGD(TAG, "Reconnecting to board %s", formatHex((uint8_t *) &bdaddr, 6));
    bluetooth->connect(bdaddr);
  }
  bluetooth->process();
}

void Wii::onEvent(std::function<void(const WiiEvent &)> eventListener) {
  this->eventListener = std::move(eventListener);
}

void Wii::disconnect(uint16_t handle) {
  // Exactly what `bluetoothctl disconnect <address>` does: a single, unconditional
  // HCI disconnect of the link. No staged per-channel L2CAP handshake, no retries.
  // The controller tears down any open L2CAP channels as part of this; we don't
  // need (and must not rely on) their own disconnect confirmations first, since
  // any intermediate half-torn-down state is exactly what let the board believe
  // the link had merely dropped and try to re-establish it.
  ESP_LOGD(TAG, "Disconnecting board handle=%u", handle);
  bluetooth->disconnect(handle);
}

void Wii::handleBoardGone_(uint16_t handle) {
  if (connectedBoards.erase(handle) > 0) {
    rejectBoardUntil_ = millis() + BOARD_RECONNECT_COOLDOWN_MS;
    this->eventListener(BalanceBoardDisconnected{.handle = handle});
  }
}

void Wii::onACLChannelClosedUnexpectedly_(uint16_t handle, uint16_t psm) {
  // The board never initiates a disconnect on its own; the only path to end a
  // session is us issuing disconnect() below. Seeing an L2CAP channel close
  // without us having asked for it is therefore abnormal (e.g. a radio glitch).
  // Bring the link fully down rather than leaving it in a half-open state,
  // which is exactly the kind of ambiguity that let the board think the link
  // had merely dropped and try to reconnect on its own.
  ESP_LOGW(TAG, "L2CAP channel psm=0x%04X closed unexpectedly for handle=%u; forcing full disconnect", psm, handle);
  bluetooth->disconnect(handle);
}

}  // namespace esphome::wii_balance_board::detail
