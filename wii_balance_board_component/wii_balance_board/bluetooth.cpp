#include "bluetooth.h"
#include "esphome/core/log.h"

#include <esp32-hal-bt.h>
#include <esp_bt.h>

#include <unordered_map>
#include <unordered_set>

#include "log.h"
#include "lowlevel_bt.h"
#include "ring_buffer.h"
#include <esp_mac.h>
#include <nvs_flash.h>
#include <nvs.h>

static const char *NVS_NAMESPACE = "wii_bb";

// Keep BT controller memory allocated on arduino-esp32 >= 3.3.7,
// which otherwise frees it at startup when no BT library is detected.
extern "C" bool btInUse() { return true; }

#define CHECK_RESULT(x) \
  if (!x) { \
    ESP_LOGE(TAG, #x " failed!"); \
  }

static const char *TAG = "bluetooth";

static_assert(CONFIG_BT_ENABLED && CONFIG_BLUEDROID_ENABLED,
              "Bluetooth is not enabled! Please run `make menuconfig` to and enable it");
static_assert(CONFIG_CLASSIC_BT_ENABLED, "Board does not support Bluetooth BR/EDR");

namespace esphome::wii_balance_board::detail {

static uint8_t g_identifier = 1;

struct L2CapConnection {
  uint16_t handle;
  uint16_t localCid;
  uint16_t psm;
  uint16_t remoteCid;
  uint16_t mtu;

  bool localConfigured;
  bool remoteConfigured;
  bool initiator;
};

class ConnectionStore {
  std::vector<L2CapConnection> l2CapConnections;

 public:
  ConnectionStore() {}
  ConnectionStore(const ConnectionStore &) = delete;
  ConnectionStore &operator=(const ConnectionStore &) = delete;

  L2CapConnection *findLocal(uint16_t handle, uint16_t localCid) {
    auto itr = std::find_if(l2CapConnections.begin(), l2CapConnections.end(),
                            [handle, localCid](const L2CapConnection &connection) {
                              return connection.handle == handle && connection.localCid == localCid;
                            });
    if (itr == l2CapConnections.end()) {
      return nullptr;
    }
    return &*itr;
  }

  L2CapConnection *findPsm(uint16_t handle, uint16_t psm) {
    auto itr = std::find_if(l2CapConnections.begin(), l2CapConnections.end(),
                            [handle, psm](const L2CapConnection &connection) {
                              return connection.handle == handle && connection.psm == psm;
                            });
    if (itr == l2CapConnections.end()) {
      return nullptr;
    }
    return &*itr;
  }

  uint16_t nextCid(uint16_t handle) {
    uint16_t nextCid = 0x0040;
    for (const auto &connection : l2CapConnections) {
      if (handle == connection.handle) {
        nextCid = std::max(nextCid, static_cast<uint16_t>(connection.localCid + 1));
      }
    }
    return nextCid;
  }

  bool remove(L2CapConnection &connection) {
    auto itr = std::remove_if(l2CapConnections.begin(), l2CapConnections.end(),
                              [&connection](const L2CapConnection &e) { return &e == &connection; });
    if (itr == l2CapConnections.end()) {
      return false;
    }
    l2CapConnections.erase(itr, l2CapConnections.end());
    return true;
  }

  bool remove(uint16_t handle) {
    auto cnt = std::erase_if(l2CapConnections, [handle](const L2CapConnection &e) { return e.handle == handle; });
    return cnt > 0;
  }

  void emplace(L2CapConnection connection) { l2CapConnections.emplace_back(std::move(connection)); }
};

struct Bluetooth::Impl {
  Bluetooth *bluetooth;
  std::array<uint8_t, 6> macAddress;
  std::function<void(Bluetooth *)> readyListener;
  std::function<void(Bluetooth *, const HCIEvent &)> hciListener;
  std::function<bool(Bluetooth *, const HCIConnectionRequest &)> connectionRequestListener = [](auto...) {
    return false;
  };

  std::function<bool(Bluetooth *, const ACLConnectionRequest &)> aclConnectionRequestListener = [](auto...) {
    return false;
  };

  std::function<void(Bluetooth *, const ACLEvent &)> aclListener;

  RingBuffer rxBuffer;
  RingBuffer txBuffer;
  ConnectionStore connections;
  bool initialized{false};
  std::unordered_set<uint64_t> discovered;
  std::unordered_set<uint64_t> connectRequests;
  std::unordered_map<uint64_t, HCIInquiryResult> nameRequests;
  std::unordered_map<uint64_t, std::array<uint8_t, 16>> linkKeys_;

  Impl(Bluetooth *bluetooth) : bluetooth(bluetooth), rxBuffer(1024), txBuffer(1024) {
    esp_read_mac(macAddress.data(), ESP_MAC_BT);
  }

  bool loadLinkKey_(uint64_t bdaddr, std::array<uint8_t, 16> *key) {
    char nvs_key[24];
    snprintf(nvs_key, sizeof(nvs_key), "lk%012llX", static_cast<unsigned long long>(bdaddr));
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
      ESP_LOGD(TAG, "NVS open for link key failed: %s", esp_err_to_name(err));
      return false;
    }
    size_t size = key->size();
    err = nvs_get_blob(handle, nvs_key, key->data(), &size);
    nvs_close(handle);
    if (err != ESP_OK || size != key->size()) {
      ESP_LOGD(TAG, "No persisted link key for %012llX: %s", static_cast<unsigned long long>(bdaddr),
               esp_err_to_name(err));
      return false;
    }
    linkKeys_[bdaddr] = *key;
    ESP_LOGD(TAG, "Loaded persisted link key for %012llX", static_cast<unsigned long long>(bdaddr));
    return true;
  }

  void saveLinkKey_(uint64_t bdaddr, const std::array<uint8_t, 16> &key) {
    linkKeys_[bdaddr] = key;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
      return;
    }
    char nvs_key[24];
    snprintf(nvs_key, sizeof(nvs_key), "lk%012llX", bdaddr);
    err = nvs_set_blob(handle, nvs_key, key.data(), 16);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "NVS set blob failed: %s", esp_err_to_name(err));
    }
    err = nvs_commit(handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(err));
    }
    std::array<uint8_t, 16> readback;
    size_t size = 16;
    err = nvs_get_blob(handle, nvs_key, readback.data(), &size);
    ESP_LOGD(TAG, "NVS readback key=%s err=%s size=%u match=%d", nvs_key, esp_err_to_name(err), (unsigned) size,
             (err == ESP_OK && size == 16 && memcmp(readback.data(), key.data(), 16) == 0));
    nvs_close(handle);
  }

  bool hasLinkKey_(uint64_t bdaddr) {
    if (linkKeys_.count(bdaddr) > 0) {
      return true;
    }
    std::array<uint8_t, 16> key;
    return loadLinkKey_(bdaddr, &key);
  }

  bool removeLinkKey_(uint64_t bdaddr) {
    char nvs_key[24];
    snprintf(nvs_key, sizeof(nvs_key), "lk%012llX", static_cast<unsigned long long>(bdaddr));
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
      return false;
    }
    err = nvs_erase_key(handle, nvs_key);
    nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) {
      ESP_LOGD(TAG, "No stored link key %s to remove", nvs_key);
      return false;
    }
    linkKeys_.erase(bdaddr);
    return true;
  }

  // Keys are collected before erasing: nvs_entry_next is not safe to use while
  // entries are being removed from the open handle.
  int removeAllLinkKeys_() {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
      return 0;
    }
    std::vector<std::string> keys;
    nvs_iterator_t iter = nullptr;
    if (nvs_entry_find_in_handle(handle, NVS_TYPE_ANY, &iter) == ESP_OK) {
      do {
        nvs_entry_info_t info;
        if (nvs_entry_info(iter, &info) != ESP_OK) {
          continue;
        }
        if (strncmp(info.key, "lk", 2) == 0) {
          keys.emplace_back(info.key);
        }
      } while (nvs_entry_next(&iter) == ESP_OK);
      nvs_release_iterator(iter);
    }
    int removed = 0;
    for (const std::string &key : keys) {
      if (nvs_erase_key(handle, key.c_str()) == ESP_OK) {
        removed++;
      }
    }
    nvs_commit(handle);
    nvs_close(handle);
    linkKeys_.clear();
    return removed;
  }

  void step() {
    while (esp_vhci_host_check_send_available()) {
      if (auto txData = txBuffer.read(0)) {
        ESP_LOGV(TAG, "TX (%u bytes): %s", (unsigned)txData.size(), formatHex(txData.data(), txData.size()));
        esp_vhci_host_send_packet(txData.data(), txData.size());
      } else {
        break;
      }
    }

    // Drain every queued RX packet per tick (not just one): our disconnect
    // handshake uses wall-clock retry timers, so leaving confirmations
    // sitting unread behind other traffic can make a graceful disconnect
    // time out and escalate to an abrupt one even though the peer already
    // replied.
    while (auto rxData = rxBuffer.read(0)) {
      const char *type;
      uint8_t typeColor;
      switch (rxData[0]) {
        case 0x04:
          if (rxData.size() < 3 || rxData.size() != static_cast<size_t>(rxData[2]) + 3) {
            ESP_LOGW(TAG, "Dropping malformed HCI event (%u bytes)", static_cast<unsigned>(rxData.size()));
            break;
          }
          type = "HCI";
          typeColor = 44;
          handleHCIEvent(rxData[1], rxData.data() + 3, rxData[2]);
          break;
        case 0x02:
          if (rxData.size() < 9) {
            ESP_LOGW(TAG, "Dropping short ACL packet (%u bytes)", static_cast<unsigned>(rxData.size()));
            break;
          }
          type = "ACL";
          typeColor = 43;
          {
            ESP_LOGV(TAG, "ACL RX (%u bytes): %s", (unsigned)rxData.size(), formatHex(rxData.data(), rxData.size()));
            uint16_t aclLength = (rxData[4] << 8) | rxData[3];
            if (rxData.size() != static_cast<size_t>(aclLength) + 5 || aclLength < 4) {
              ESP_LOGW(TAG, "Dropping malformed ACL frame size=%u acl_length=%u", static_cast<unsigned>(rxData.size()),
                       aclLength);
              break;
            }
            uint16_t handle = ((rxData[2] & 0x0F) << 8) | rxData[1];
            uint8_t packetBoundaryFlag = (rxData[2] & 0x30) >> 4;  // Packet_Boundary_Flag
            uint8_t broadcastFlag = (rxData[2] & 0xC0) >> 6;       // Broadcast_Flag

            if (packetBoundaryFlag != 0b10) {
              ESP_LOGE(TAG, "unsupported packet_boundary_flag = 0b%02B", packetBoundaryFlag);
              break;
            }

            if (broadcastFlag != 0b00) {
              ESP_LOGE(TAG, "unsupported broadcast_flag 0b%02B", broadcastFlag);
              break;
            }

            uint16_t len = (rxData[6] << 8) | rxData[5];
            if (len != aclLength - 4) {
              ESP_LOGW(TAG, "Dropping malformed L2CAP frame length=%u acl_payload=%u", len, aclLength - 4);
              break;
            }
            uint16_t channelId = (rxData[8] << 8) | rxData[7];
            if (len > 0) {
              handleACLEvent(rxData[9], handle, channelId, rxData.data() + 9, len);
            }
          }
          break;
        default:
          type = "ERR";
          typeColor = 41;
          break;
      }

      // ESP_LOGD(TAG, "[%s] RX> %s", type, formatHex(rxData.data(), rxData.size()));
    }
  }

  // HCI
  void handleHCICommandComplete(uint8_t *data, size_t len) {
    if (data[1] == 0x03 && data[2] == 0x0C) {  // reset
      if (data[3] == 0x00) {
        CHECK_RESULT(enqueue_cmd_read_bd_addr(txBuffer));
      } else {
        ESP_LOGE(TAG, "Reset failed");
      }
    } else if (data[1] == 0x09 && data[2] == 0x10) {  // read_bd_addr
      if (data[3] == 0x00) {
        char name[] = "ESP32-BT-WIIP";
        CHECK_RESULT(enqueue_cmd_write_local_name(txBuffer, (uint8_t *) name, sizeof(name)));
      } else {
        ESP_LOGE(TAG, "read_bd_addr failed.");
      }
    } else if (data[1] == 0x13 && data[2] == 0x0C) {  // write_local_name
      if (data[3] == 0x00) {
        uint8_t cod[3] = {0x04, 0x05, 0x00};
        CHECK_RESULT(enqueue_cmd_write_class_of_device(txBuffer, cod));
      } else {
        ESP_LOGE(TAG, "write_local_name failed.");
      }
    } else if (data[1] == 0x24 && data[2] == 0x0C) {  // write_class_of_device
      if (data[3] == 0x00) {
        CHECK_RESULT(enqueue_cmd_write_default_link_policy(txBuffer, 0x0001));
      } else {
        ESP_LOGE(TAG, "write_class_of_device failed.");
      }
    } else if (data[1] == 0x0F && data[2] == 0x08) {  // write_default_link_policy
      if (data[3] == 0x00) {
        ESP_LOGD(TAG, "Role switch enabled in default link policy");
      } else {
        ESP_LOGW(TAG, "write_default_link_policy failed status=0x%02X", data[3]);
      }
      // Widen the page scan window well beyond the default (~11ms out of every 1.28s, <1%
      // duty cycle) to reduce the chance of missing an incoming page from the board while
      // sharing the radio with Wi-Fi. Kept modest (~25% duty cycle): a 100% and even a 50%
      // duty cycle combined with esp_coex_preference_set(ESP_COEX_PREFER_BT) completely broke
      // Wi-Fi association (probe requests never completed at all) -- the ESP32 has only one
      // shared antenna, so BT/Wi-Fi coexistence is a hard time-slice tradeoff, not free. The
      // explicit coexistence preference override was reverted entirely for the same reason.
      CHECK_RESULT(enqueue_cmd_write_page_scan_activity(txBuffer, 0x0800, 0x0200));
    } else if (data[1] == 0x1C && data[2] == 0x0C) {  // write_page_scan_activity
      if (data[3] == 0x00) {
        ESP_LOGD(TAG, "Page scan window widened");
      } else {
        ESP_LOGW(TAG, "write_page_scan_activity failed status=0x%02X", data[3]);
      }
      CHECK_RESULT(enqueue_cmd_write_page_scan_type(txBuffer, 1));  // 1 = interlaced page scan
    } else if (data[1] == 0x47 && data[2] == 0x0C) {  // write_page_scan_type
      if (data[3] == 0x00) {
        ESP_LOGD(TAG, "Interlaced page scan enabled");
      } else {
        ESP_LOGW(TAG, "write_page_scan_type failed status=0x%02X", data[3]);
      }
      CHECK_RESULT(enqueue_cmd_write_scan_enable(txBuffer, 3));
    } else if (data[1] == 0x1A && data[2] == 0x0C) {  // write_scan_enable
      if (data[3] == 0x00) {
        bool was_initialized = initialized;
        initialized = true;
        if (!was_initialized) {
          readyListener(bluetooth);
        } else {
          ESP_LOGD(TAG, "Page and inquiry scan re-enabled");
        }
      } else {
        ESP_LOGE(TAG, "write_scan_enable failed status=0x%02X", data[3]);
      }
    }
  }

  void handleHCICommandStatusEvent(uint8_t *data, size_t len) {
    uint16_t opcode = (uint16_t)(data[3] << 8 | data[2]);
    if (opcode == (0x0005 | HCI_GRP_LINK_CONT_CMDS)) {
      ESP_LOGD(TAG, "Create_Connection status=0x%02X", data[0]);
    } else if (opcode == (0x0011 | HCI_GRP_LINK_CONT_CMDS)) {
      ESP_LOGD(TAG, "Authentication_Requested status=0x%02X", data[0]);
    } else if (opcode == (0x0001 | HCI_GRP_LINK_CONT_CMDS)) {
      if (data[0] == 0x00) {
        hciListener(bluetooth, HCIInquiryStarted{});
      } else {
        log_e("Failed to start inquiry, error=%02X", data[0]);
      }
    }
  }

  void handleHCIInqueryResult(uint8_t *data, size_t len) {
    uint8_t num = data[0];
    for (uint8_t i = 0; i < num; ++i) {
      int pos = 1 + (6 + 1 + 2 + 3 + 2) * i;
      uint64_t bdaddr = *(const uint64_t *) (data + pos) & 0xFFFFFFFFFFFFull;
      uint32_t cod = (data[pos + 9] << 16) | (data[pos + 10] << 8) | data[pos + 11];
      if (discovered.emplace(bdaddr).second) {
        HCIInquiryResult res{
            .bdaddr = bdaddr,
            .psrm = data[pos + 6],
            .classOfDevice = cod,
            .clkOffset = static_cast<uint16_t>(((0x80 | data[pos + 12]) << 8) | (data[pos + 13])),
        };
        hciListener(bluetooth, res);
      }
    }
  }

  void handleHCIInqueryComplete(uint8_t *data, size_t len) {
    hciListener(bluetooth, HCIInquiryComplete{});
    discovered.clear();
  }

  void handleHCIDisconnect(uint8_t *data, size_t len) {
    uint8_t status = data[0];
    uint16_t handle = data[2] << 8 | data[1];
    uint8_t reason = data[3];
    ESP_LOGD(TAG, "Disconnect handle=%d status=0x%02X reason=0x%02X", handle, status, reason);
    if (status == 0x00) {
      hciListener(bluetooth, HCIDisconnected{.handle = handle, .reason = reason});
      connections.remove(handle);
      // ESP32 controller may drop page scan after disconnect; re-enable it.
      enqueue_cmd_write_scan_enable(txBuffer, 3);
    }
  }

  void handleHCIRemoteNameRequestComplete(uint8_t *data, size_t len) {
    uint8_t status = data[0];
    char *name = (char *) (data + 7);
    uint64_t bdaddr = *(const uint64_t *) (data + 1) & 0xFFFFFFFFFFFFull;
    auto &inquiry = nameRequests.at(bdaddr);
    hciListener(bluetooth, HCIRemoteName{.inquiry =
                                             HCIInquiryResult{
                                                 .bdaddr = inquiry.bdaddr,
                                                 .psrm = inquiry.psrm,
                                                 .classOfDevice = inquiry.classOfDevice,
                                                 .clkOffset = inquiry.clkOffset,
                                             },
                                         .remoteName = {name}});
    nameRequests.erase(bdaddr);
  }

  void handleHCIConnectionComplete(uint8_t *data, size_t len) {
    uint8_t status = data[0];
    uint16_t handle = data[2] << 8 | data[1];
    uint64_t bdaddr = *(const uint64_t *) (data + 3) & 0xFFFFFFFFFFFFull;
    if (status == 0x00) {
      hciListener(bluetooth, HCIConnectionEstablished{
                                  .bdaddr = bdaddr, .handle = handle, .accepted = !connectRequests.contains(bdaddr)});
    } else {
      hciListener(
          bluetooth,
          HCIConnectionFailed{
              .bdaddr = bdaddr, .handle = handle, .reason = status, .accepted = !connectRequests.contains(bdaddr)});
      if (status == 0x0F) {
        ESP_LOGD(TAG, "Connection request rejected; re-enabling page scan");
      } else {
          ESP_LOGW(TAG, "Connection complete failed status=0x%02X, re-enabling page scan", status);
      }
      CHECK_RESULT(enqueue_cmd_write_scan_enable(txBuffer, 3));
    }
    connectRequests.erase(bdaddr);
  }

  void handleHCIConnectionRequest(uint8_t *data, size_t len) {
    if (len < 10) {
      ESP_LOGW(TAG, "Ignoring short HCI connection request (%u bytes)", static_cast<unsigned>(len));
      return;
    }
    uint64_t bdaddr = *(const uint64_t *) (data) &0xFFFFFFFFFFFFull;
    uint32_t cod = (data[6] << 16) | (data[7] << 8) | data[8];
    uint8_t link_type = data[9];
    ESP_LOGD(TAG, "HCI connection request CoD=0x%06X link_type=0x%02X", static_cast<unsigned>(cod), link_type);

    if (connectionRequestListener(bluetooth, HCIConnectionRequest{.bdaddr = bdaddr, .classOfDevice = cod})) {
      ESP_LOGD(TAG, "Accepting connection from %s role=0x00 (become master)", formatHex((uint8_t *)&bdaddr, 6));
      CHECK_RESULT(enqueue_cmd_accept_connection(txBuffer, bdaddr, 0x00));
    } else {
      ESP_LOGD(TAG, "Rejecting connection from %s", formatHex((uint8_t *)&bdaddr, 6));
      CHECK_RESULT(enqueue_cmd_reject_connection(txBuffer, bdaddr, 0x0F));
    }
  }

  void handleHCIPINRequest(uint8_t *data, size_t len) {
    uint64_t bdaddr = *(const uint64_t *) (data) &0xFFFFFFFFFFFFull;

    hciListener(bluetooth, HCIPINRequest{.bdaddr = bdaddr});
  }

  void handleHCILinkKeyRequest(uint8_t *data, size_t len) {
    uint64_t bdaddr = *(const uint64_t *) (data) &0xFFFFFFFFFFFFull;

    auto it = linkKeys_.find(bdaddr);
    if (it != linkKeys_.end()) {
      ESP_LOGD(TAG, "Link key found, replying for %s", formatHex((uint8_t *)&bdaddr, 6));
      CHECK_RESULT(enqueue_cmd_link_key_reply(txBuffer, bdaddr, it->second.data()));
      return;
    }
    std::array<uint8_t, 16> key;
    if (loadLinkKey_(bdaddr, &key)) {
      CHECK_RESULT(enqueue_cmd_link_key_reply(txBuffer, bdaddr, key.data()));
      return;
    }
    ESP_LOGD(TAG, "No stored link key for %s", formatHex((uint8_t *)&bdaddr, 6));

    uint8_t keyType = data[22];

    hciListener(bluetooth, HCILinkKeyRequest{
                               .bdaddr = bdaddr,
                               .keyType = keyType,
                               .linkKeyData = data + 6,
                               .size = 16,
                           });
  }

  void handleHCILinkKeyNotification(uint8_t *data, size_t len) {
    uint64_t bdaddr = *(const uint64_t *)data & 0xFFFFFFFFFFFFull;
    std::array<uint8_t, 16> key;
    memcpy(key.data(), data + 6, 16);
    ESP_LOGD(TAG, "Link key notification for BD_ADDR %s, keyType=0x%02X", formatHex((uint8_t *)&bdaddr, 6),
             len >= 23 ? data[22] : 0xFF);
    saveLinkKey_(bdaddr, key);
  }

  void dumpHex(const char *label, const uint8_t *data, size_t len) {
    char buf[256];
    size_t pos = 0;
    for (size_t i = 0; i < len && pos < sizeof(buf) - 4; i++) {
      auto n = snprintf(buf + pos, sizeof(buf) - pos, "%02X ", data[i]);
      if (n > 0) pos += n;
    }
    ESP_LOGV(TAG, "%s [%d] %s", label, len, buf);
  }

  void handleHCIEvent(uint8_t eventCode, uint8_t *data, size_t len) {
    dumpHex("HCI_EVT", data - 2, len + 2);  // include event code + param length
    switch (eventCode) {
      case 0x0F:
        handleHCICommandStatusEvent(data, len);
        break;
      case 0x0E:
        handleHCICommandComplete(data, len);
        break;
      case 0x02:
        handleHCIInqueryResult(data, len);
        break;
      case 0x01:
        handleHCIInqueryComplete(data, len);
        break;
      case 0x05:
        handleHCIDisconnect(data, len);
        break;
      case 0x07:
        handleHCIRemoteNameRequestComplete(data, len);
        break;
      case 0x03:
        handleHCIConnectionComplete(data, len);
        break;
      case 0x04:
        handleHCIConnectionRequest(data, len);
        break;
      case 0x17:
        handleHCILinkKeyRequest(data, len);
        break;
      case 0x16:
        handleHCIPINRequest(data, len);
        break;
      case 0x18:
        handleHCILinkKeyNotification(data, len);
        break;
      case 0x12: {
        uint64_t bdaddr = *(const uint64_t *) (data + 1) &0xFFFFFFFFFFFFull;
        ESP_LOGD(TAG, "Role Changed status=0x%02X %s new_role=0x%02X", data[0],
                 formatHex((uint8_t *) &bdaddr, 6), data[7]);
        hciListener(bluetooth, HCIRoleChanged{.bdaddr = bdaddr, .status = data[0], .newRole = data[7]});
        break;
      }
      case 0x06: {
        uint8_t status = data[0];
        uint16_t handle = (uint16_t)(data[2] << 8 | data[1]);
        ESP_LOGD(TAG, "Authentication Complete handle=%d status=0x%02X", handle, status);
        hciListener(bluetooth, HCIAuthComplete{.handle = handle, .status = status});
        break;
      }
      case 0x08: {
        uint8_t status = data[0];
        uint16_t handle = (uint16_t)(data[2] << 8 | data[1]);
        uint8_t enabled = data[3];
        ESP_LOGD(TAG, "Encryption Change handle=%d status=0x%02X enabled=%d", handle, status, enabled);
        hciListener(bluetooth, HCIEncryptionChange{.handle = handle, .status = status});
        break;
      }
    }
  }

  void sendHCIReset() { CHECK_RESULT(enqueue_cmd_reset(txBuffer)); }

  void sendHCIDisconnect(uint16_t handle) {
    constexpr uint8_t reason = 0x13;
    ESP_LOGD(TAG, "Queuing Disconnect handle=%u reason=0x%02X", handle, reason);
    CHECK_RESULT(enqueue_cmd_disconnect(txBuffer, handle, reason));
  }

  void sendHCIScan() {
    if (!initialized) {
      ESP_LOGE(TAG, "Cannot sync, bluetooth not initialized");
      return;
    }

    uint8_t timeout = 0x10;  // Sync for 20.48 seconds (0x10 * 1.28s)

    CHECK_RESULT(enqueue_cmd_inquiry(txBuffer, 0x9E8B33, timeout, 0x00));
  }

  void sendHCIScanCancel() {
    if (!initialized) {
      ESP_LOGE(TAG, "Cannot sync, bluetooth not initialized");
      return;
    }

    uint8_t timeout = 0x10;  // Sync for 20.48 seconds (0x10 * 1.28s)

    CHECK_RESULT(enqueue_cmd_inquiry_cancel(txBuffer));
  }

  void sendHCIRequestRemoteName(const HCIInquiryResult &result) {
    nameRequests.emplace(result.bdaddr, result);
    CHECK_RESULT(enqueue_cmd_remote_name_request(txBuffer, result.bdaddr, result.psrm, result.clkOffset));
  }

  void sendHCIConnect(const HCIInquiryResult &result) {
    ESP_LOGD(TAG, "Queuing Create_Connection to %s", formatHex((uint8_t *)&result.bdaddr, 6));
    connectRequests.emplace(result.bdaddr);
    CHECK_RESULT(enqueue_cmd_create_connection(txBuffer, result.bdaddr, 0x0008, result.psrm, result.clkOffset, 0x01));
  }

  void sendHCINegativeReply(uint64_t bdaddr) { CHECK_RESULT(enqueue_cmd_negative_reply(txBuffer, bdaddr)); }

  void sendLinkKeyReply_(uint64_t bdaddr, const uint8_t *key) { CHECK_RESULT(enqueue_cmd_link_key_reply(txBuffer, bdaddr, key)); }

  void sendHCIPINReply(uint64_t bdaddr, uint8_t *pinData, size_t len) {
    if (len > 16) {
      ESP_LOGE(TAG, "PIN too long, max 16 characters");
      return;
    }
    CHECK_RESULT(enqueue_cmd_pin_reply(txBuffer, bdaddr, pinData, len));
  }

  void sendHCIAuth(uint16_t handle) {
    ESP_LOGD(TAG, "Queuing Authentication_Requested handle=%d", handle);
    CHECK_RESULT(enqueue_cmd_auth_request(txBuffer, handle));
  }

  void sendHCISetEncryption(uint16_t handle) {
    ESP_LOGD(TAG, "Queuing Set_Connection_Encryption handle=%d enable=1", handle);
    CHECK_RESULT(enqueue_cmd_set_encryption(txBuffer, handle, 0x01));
  }

  void sendHCISwitchRole(uint64_t bdaddr) {
    ESP_LOGD(TAG, "Queuing Switch_Role to master for %s", formatHex((uint8_t *)&bdaddr, 6));
    CHECK_RESULT(enqueue_cmd_switch_role(txBuffer, bdaddr, 0x00));
  }

  // ACL
  void handleL2ConfigurationRequest(uint16_t handle, uint8_t *data) {
    uint8_t identifier = data[1];
    uint16_t len = (data[3] << 8) | data[2];
    uint16_t destinationCid = (data[5] << 8) | data[4];
    uint16_t flags = (data[7] << 8) | data[6];

    if (flags != 0x0000) {
      ESP_LOGE(TAG, "Unsupported flags %04X", flags);
      return;
    }

    if (len != 0x08) {
      ESP_LOGE(TAG, "Unexpected configuration length %04X", len);
      return;
    }

    L2CapConnection *connection = connections.findLocal(handle, destinationCid);
    if (connection == nullptr) {
      ESP_LOGW(TAG, "Unexpected configuration requestion");
      return;
    }

    if (data[8] == 0x01 && data[9] == 0x02) {  // MTU
      uint16_t mtu = (data[11] << 8) | data[10];
      connection->mtu = mtu;
      uint8_t packetBoundaryFlag = 0b10;  // Packet_Boundary_Flag
      uint8_t broadcastFlag = 0b00;       // Broadcast_Flag
      uint16_t channelId = 0x0001;
      uint16_t sourceCid = connection->remoteCid;
      uint8_t data[] = {
          0x05,        // CONFIGURATION RESPONSE
          identifier,  // Identifier
          0x0A,
          0x00,  // Length: 0x000A
          (uint8_t) (sourceCid & 0xFF),
          (uint8_t) (sourceCid >> 8),  // Source CID
          0x00,
          0x00,  // Flags
          0x00,
          0x00,  // Res
          0x01,
          0x02,
          (uint8_t) (mtu & 0xFF),
          (uint8_t) (mtu >> 8)  // type=01 len=02 value=xx xx
      };

      uint16_t dataLen = 14;
      CHECK_RESULT(enqueue_acl_l2cap_single_packet(txBuffer, handle, packetBoundaryFlag, broadcastFlag, channelId, data,
                                                   dataLen));
      connection->remoteConfigured = true;
      ESP_LOGD(TAG, "L2CAP config request handle=%d dcid=0x%04X mtu=%d initiator=%d", handle, destinationCid, mtu,
               connection->initiator);
      if (connection->remoteConfigured && connection->localConfigured) {
        ESP_LOGD(TAG, "L2CAP established handle=%d psm=0x%04X accepted=%d", handle, connection->psm,
                 !connection->initiator);
        aclListener(bluetooth, ACLConnectionEstablished{
                                    .handle = handle,
                                    .sourceCid = sourceCid,
                                    .psm = connection->psm,
                                    .accepted = !connection->initiator,
                                });
      }
    }
  }

  void handleL2DisconnectRequest(uint16_t handle, uint8_t *data) {
    uint8_t identifier = data[1];
    uint16_t destinationCid = (data[5] << 8) | data[4];
    uint16_t sourceCid = (data[7] << 8) | data[6];
    uint32_t key = handle << 16 | destinationCid;

    L2CapConnection *connection = connections.findLocal(handle, destinationCid);
    if (connection == nullptr) {
      // Send command reject rsp
      return;
    }
    ESP_LOGD(TAG, "Sending disconnect response");
    if (connection->remoteCid == sourceCid) {
      uint16_t psm = connection->psm;
      uint8_t response[] = {
          0x07,        // Disconnect response
          identifier,  // Identifier
          0x04,
          0x00,  // Length: 0x0004
          (uint8_t) (connection->localCid & 0xFF),
          (uint8_t) (connection->localCid >> 8),  // Destination CID
          (uint8_t) (connection->remoteCid & 0xFF),
          (uint8_t) (connection->remoteCid >> 8),  // Source CID
      };

      sendL2DataChannel(handle, 0x0001, response, 8);
      connections.remove(*connection);
      // The board is allowed to close a channel on its own (e.g. its own idle
      // power-off timer); tell the app layer the same way we would for a
      // response to our own disconnect request, so it can finish tearing
      // down rather than leaving a stale ACL link and app-level state.
      aclListener(bluetooth, ACLDisconnected{.handle = handle, .psm = psm});
    } else {
      ESP_LOGD(TAG, "Mismatch");
    }
  }

  void handleL2ConnectionResponse(uint16_t handle, uint8_t *data) {
    dumpHex("L2CAP_CON_RSP", data, ((data[3] << 8) | data[2]) + 4);
    uint8_t identifier = data[1];
    uint16_t len = (data[3] << 8) | data[2];
    uint16_t destinationCid = (data[5] << 8) | data[4];
    uint16_t sourceCid = (data[7] << 8) | data[6];
    uint16_t result = (data[9] << 8) | data[8];
    uint16_t status = (data[11] << 8) | data[10];

    auto *connection = connections.findLocal(handle, sourceCid);
    if (connection == nullptr) {
      ESP_LOGW(TAG, "Received unexpected L2Cap Connection response, ignoring");
      return;
    }

    if (result == 0x0000) {  // Connection established, initiate configuration
      connection->remoteCid = destinationCid;
      ESP_LOGD(TAG, "L2CAP connect response OK handle=%d psm=0x%04X dcid=0x%04X, sending config", handle,
               connection->psm, destinationCid);
      sendL2Configure(handle, destinationCid, connection->mtu);
    } else if (result >= 0x0002) {  // Connection failed
      ESP_LOGD(TAG, "L2CAP connect response FAILED handle=%d psm=0x%04X result=0x%04X status=0x%04X", handle, connection->psm,
               result, status);
      aclListener(bluetooth, ACLConnectionFailed{
                                 .handle = handle,
                                 .sourceCid = sourceCid,
                                 .psm = connection->psm,
                             });
      connections.remove(*connection);
    }
  }

  void handleL2ConfigurationResponse(uint16_t handle, uint8_t *data) {
    uint16_t sourceCid = (data[5] << 8) | data[4];
    auto *connection = connections.findLocal(handle, sourceCid);
    if (connection == nullptr) {
      ESP_LOGW(TAG, "L2CAP config response for unknown connection handle=%d scid=0x%04X", handle, sourceCid);
      return;
    }

    connection->localConfigured = true;
    if (connection->localConfigured && connection->remoteConfigured) {
      ESP_LOGD(TAG, "L2CAP established handle=%d psm=0x%04X accepted=%d", handle, connection->psm,
               !connection->initiator);
      aclListener(bluetooth, ACLConnectionEstablished{
                                 .handle = handle,
                                 .sourceCid = sourceCid,
                                 .psm = connection->psm,
                                 .accepted = !connection->initiator,
                             });
    }
  }

  void handleL2DisconnectResponse(uint16_t handle, uint8_t *data) {
    uint16_t sourceCid = (data[7] << 8) | data[6];
    auto *connection = connections.findLocal(handle, sourceCid);
    if (connection) {
      aclListener(bluetooth, ACLDisconnected{
                                 .handle = handle,
                                 .psm = connection->psm,
                             });
      connections.remove(*connection);
    }
  }

  void handleL2ConnectionRequest(uint16_t handle, uint8_t *data) {
    dumpHex("L2CAP_CON_REQ", data, ((data[3] << 8) | data[2]) + 4);
    uint16_t sourceCid = (data[7] << 8) | data[6];
    uint16_t psm = (data[5] << 8) | data[4];
    ESP_LOGD(TAG, "L2CAP connection request handle=%d psm=0x%04X scid=0x%04X", handle, psm, sourceCid);
    bool accepted = aclConnectionRequestListener(
        bluetooth, ACLConnectionRequest{.handle = handle, .sourceCid = sourceCid, .psm = psm});
    auto localCid = connections.nextCid(handle);
    if (accepted) {
      connections.emplace(L2CapConnection{
          .handle = handle,
          .localCid = localCid,
          .psm = psm,
          .remoteCid = sourceCid,
          .mtu = 0x00B9,
          .localConfigured = false,
          .remoteConfigured = false,
          .initiator = false,
      });
    }
    uint16_t result = accepted ? 0x00 : 0x04;  // Connection refused if idx == -1.
    uint8_t response[] = {
        0x03,
        data[1],  // Request identifier
        0x08,
        0x00,
        (uint8_t) (localCid & 0xFF),
        (uint8_t) (localCid >> 8),
        (uint8_t) (sourceCid & 0xFF),
        (uint8_t) (sourceCid >> 8),
        (uint8_t) (result & 0xFF),
        (uint8_t) (result >> 8),
        0x00,
        0x00,  // No status
    };
    sendL2DataChannel(handle, 0x0001, response, 12);
    if (accepted) {  // Send config request
      sendL2Configure(handle, sourceCid, 0x00B9);
    }
  }

  void handleACLEvent(uint8_t event, uint16_t handle, uint16_t channelId, uint8_t *data, size_t len) {
    if (channelId == 0x0001) {
      dumpHex("ACL_RX_SIG", data, len);
    }
    switch (event) {
      case 0x02:
        handleL2ConnectionRequest(handle, data);
        break;
      case 0x03:
        handleL2ConnectionResponse(handle, data);
        break;
      case 0x04:
        handleL2ConfigurationRequest(handle, data);
        break;
      case 0x05:
        handleL2ConfigurationResponse(handle, data);
        break;
      case 0x06:
        handleL2DisconnectRequest(handle, data);
        break;
      case 0x07:
        handleL2DisconnectResponse(handle, data);
        break;
      default:
        if (channelId != 0x0001) {
          dumpHex("ACL_RX_DATA", data, len);
        }
        aclListener(bluetooth, ACLData{
                                   .handle = handle,
                                   .channelId = channelId,
                                   .data = data,
                                   .len = len,
                               });
        break;
    }
  }

  void sendL2Configure(uint16_t handle, uint16_t destinationCid, uint16_t mtu) {
    uint8_t data[] = {
        0x04,            // CONFIGURATION REQUEST
        g_identifier++,  // Identifier
        0x08,
        0x00,  // Length: 0x0008
        (uint8_t) (destinationCid & 0xFF),
        (uint8_t) (destinationCid >> 8),  // Destination CID
        0x00,
        0x00,  // Flags
        0x01,
        0x02,
        (uint8_t) (mtu & 0xFF),
        (uint8_t) (mtu >> 8)  // type=01 len=02 value=2 bytes mtu
    };

    sendL2DataChannel(handle, 0x0001, data, 12);
  }

  void sendL2Connect(uint16_t connection_handle, uint16_t psm, uint16_t mtu) {
    uint16_t localCid = connections.nextCid(connection_handle);
    uint8_t data[] = {0x02,          // CONNECTION REQUEST
                      g_identifier,  // Identifier
                      0x04,
                      0x00,  // Length:     0x0004
                      (uint8_t) (psm & 0xFF),
                      (uint8_t) (psm >> 8),
                      (uint8_t) (localCid & 0xFF),
                      (uint8_t) (localCid >> 8)};
    uint16_t data_len = 8;

    sendL2DataChannel(connection_handle, 0x0001, data, 8);

    ESP_LOGD(TAG, "Queued L2CAP connect handle=%d psm=0x%04X mtu=0x%04X lcid=0x%04X", connection_handle, psm, mtu,
             localCid);

    connections.emplace(L2CapConnection{
        .handle = connection_handle,
        .localCid = localCid,
        .psm = psm,
        .remoteCid = 0,
        .mtu = mtu,
        .localConfigured = false,
        .remoteConfigured = false,
        .initiator = true,
    });
    g_identifier++;
  }

  void sendL2Data(uint16_t handle, uint16_t psm, uint8_t *data, size_t len) {
    uint32_t hp = (handle << 16) | psm;
    auto *connection = connections.findPsm(handle, psm);
    if (connection == nullptr) {
      ESP_LOGE(TAG, "Cannot send L2 data, handle/psm connection not found");
      return;
    }
    sendL2DataChannel(handle, connection->remoteCid, data, len);
  }

  bool sendL2Disconnect(uint16_t handle, uint16_t psm) {
    auto *connection = connections.findPsm(handle, psm);
    if (connection) {
      uint8_t data[] = {
          0x06,            // Disconnect REQUEST
          g_identifier++,  // Identifier
          0x04,
          0x00,  // Length: 0x0004
          (uint8_t) (connection->remoteCid & 0xFF),
          (uint8_t) (connection->remoteCid >> 8),
          (uint8_t) (connection->localCid & 0xFF),
          (uint8_t) (connection->localCid >> 8),
      };

      sendL2DataChannel(handle, 0x0001, data, 8);
      return true;
    }
    ESP_LOGD(TAG, "No L2CAP channel handle=%u psm=0x%04X to disconnect; already closed", handle, psm);
    return false;
  }

  void sendL2DataChannel(uint16_t handle, uint16_t channelId, uint8_t *data, size_t len) {
    if (channelId == 0x0001) {
      dumpHex("ACL_TX_SIG", data, len);
    }
    uint8_t packetBoundaryFlag = 0b10;  // Packet_Boundary_Flag
    uint8_t broadcastFlag = 0b00;       // Broadcast_Flag

    CHECK_RESULT(
        enqueue_acl_l2cap_single_packet(txBuffer, handle, packetBoundaryFlag, broadcastFlag, channelId, data, len));
  }
};

std::function<int(uint8_t *data, size_t len)> gListener;

static void sendReady() {}

static int recv(uint8_t *data, uint16_t len) { return gListener(data, len); }

static const esp_vhci_host_callback_t callback = {sendReady, recv};

Bluetooth::Bluetooth() : m_impl(std::make_unique<Bluetooth::Impl>(this)) {
  if (!btStart()) {
    ESP_LOGE(TAG, "Failed to initialize Bluetooth");
    return;
  }

  auto *impl = m_impl.get();
  gListener = [impl](uint8_t *data, size_t len) {
    if (auto buffer = impl->rxBuffer.allocate(len, portMAX_DELAY)) {
      memcpy(buffer.data(), data, len);
      return ESP_OK;
    }
    ESP_LOGW(TAG, "Buffer error, dropping packets.");
    return ESP_OK;
  };

  esp_vhci_host_register_callback(&callback);
  m_impl->sendHCIReset();
}

Bluetooth::~Bluetooth() { ESP_LOGD(TAG, "Shut down"); }

void Bluetooth::onReady(const std::function<void(Bluetooth *)> &listener) { m_impl->readyListener = listener; }

void Bluetooth::process() { m_impl->step(); }

// HCI
void Bluetooth::onHCIEvent(const std::function<void(Bluetooth *, const HCIEvent &)> &listener) {
  m_impl->hciListener = listener;
}

void Bluetooth::onHCIConnectionRequest(const std::function<bool(Bluetooth *, const HCIConnectionRequest &)> &listener) {
  m_impl->connectionRequestListener = listener;
}

void Bluetooth::requestRemoteName(const HCIInquiryResult &result) { m_impl->sendHCIRequestRemoteName(result); }

void Bluetooth::connect(const HCIInquiryResult &result) { m_impl->sendHCIConnect(result); }
void Bluetooth::connect(uint64_t bdaddr) { m_impl->sendHCIConnect(HCIInquiryResult{.bdaddr = bdaddr}); }

void Bluetooth::scan(bool enable) {
  if (enable) {
    m_impl->sendHCIScan();
  } else {
    m_impl->sendHCIScanCancel();
  }
}

void Bluetooth::disconnect(uint16_t handle) { m_impl->sendHCIDisconnect(handle); }

// ACL
void Bluetooth::l2cap_connect(uint16_t handle, uint16_t psm, uint16_t mtu) { m_impl->sendL2Connect(handle, psm, mtu); }

void Bluetooth::onACLEvent(const std::function<void(Bluetooth *, const ACLEvent &)> &listener) {
  m_impl->aclListener = listener;
}

void Bluetooth::auth(uint16_t handle) { m_impl->sendHCIAuth(handle); }

void Bluetooth::setEncryption(uint16_t handle) { m_impl->sendHCISetEncryption(handle); }

void Bluetooth::switch_role(uint64_t bdaddr) { m_impl->sendHCISwitchRole(bdaddr); }

void Bluetooth::negativeReply(uint64_t bdaddr) { m_impl->sendHCINegativeReply(bdaddr); }

void Bluetooth::sendPinReply(uint64_t bdaddr, uint8_t *pinData, size_t len) {
  m_impl->sendHCIPINReply(bdaddr, pinData, len);
}

void Bluetooth::sendLinkKeyReply(uint64_t bdaddr, const uint8_t *key) {
  m_impl->sendLinkKeyReply_(bdaddr, key);
}

bool Bluetooth::hasLinkKey(uint64_t bdaddr) { return m_impl->hasLinkKey_(bdaddr); }

bool Bluetooth::removeLinkKey(uint64_t bdaddr) { return m_impl->removeLinkKey_(bdaddr); }

int Bluetooth::removeAllLinkKeys() { return m_impl->removeAllLinkKeys_(); }

void Bluetooth::onACLConnectionRequest(const std::function<bool(Bluetooth *, const ACLConnectionRequest &)> &listener) {
  m_impl->aclConnectionRequestListener = listener;
}

bool Bluetooth::l2cap_disconnect(uint16_t handle, uint16_t psm) { return m_impl->sendL2Disconnect(handle, psm); }

void Bluetooth::l2send_data(uint16_t handle, uint16_t psm, uint8_t *data, size_t len) {
  m_impl->sendL2Data(handle, psm, data, len);
}

std::span<uint8_t, 6> Bluetooth::macAddress() { return m_impl->macAddress; }

}  // namespace esphome::wii_balance_board::detail
