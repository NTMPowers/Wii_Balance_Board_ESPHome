#pragma once
#include "bluetooth.h"
#include "esphome/core/hal.h"
#include <optional>
#include <unordered_set>
#include <unordered_map>
#include <memory>

namespace esphome::wii_balance_board::detail {

struct BalanceBoardConnected {
  uint16_t handle;
  uint64_t bdaddr;
};

struct BalanceBoardDisconnected {
  uint16_t handle;
};

// The board's 0 kg points and reference temperature have been rewritten. Measurements
// are only meaningful once this arrives.
struct BalanceBoardTared {
  uint16_t handle;
  bool ok;
};

struct BalanceBoardData {
  uint16_t handle;
  uint16_t tr;
  uint16_t br;
  uint16_t tl;
  uint16_t bl;
  uint8_t referenceTemperature;
  uint8_t temperature;
  uint8_t batteryLevel;
};

struct ScanStarted {};

struct ScanStopped {};

using WiiEvent = std::variant<BalanceBoardConnected, BalanceBoardDisconnected, BalanceBoardData, BalanceBoardTared,
                              ScanStarted, ScanStopped>;

class Wii {
  struct BalanceBoard;
  Bluetooth *bluetooth;
  std::unordered_map<uint16_t, std::unique_ptr<BalanceBoard>> connectedBoards;
  std::unordered_map<uint16_t, uint64_t> handleToBdaddr;
  std::unordered_set<uint16_t> pendingPSM13;
  std::unordered_set<uint16_t> pendingEncryption;
  std::unordered_set<uint16_t> initiatorHandles;
  // Boards whose accept-time role switch lost its LMP race. The board only opens
  // L2CAP as link slave, so the role is forced at encryption time.
  std::unordered_set<uint64_t> needsRoleSwitch;
  std::optional<uint64_t> pendingReconnect;
  bool reconnecting{false};
  uint64_t rejectBoardUntil_{0};
  // Set on the first cooldown refusal, cleared when a new cooldown window opens,
  // so a board that keeps re-paging only logs one INFO line per window.
  bool cooldownLogged_{false};
  // A board may only connect while it has a stored link key. This window is opened
  // by sync() to let an unpaired board in long enough to pair.
  uint64_t pairingAllowedUntil_{0};
  uint64_t lastUnpairedRefusalLogMs_{0};
  std::function<void(const WiiEvent &)> eventListener;

 public:
  Wii(Bluetooth *bluetooth);
  ~Wii();
  Wii(const Wii &) = delete;
  Wii &operator=(const Wii &) = delete;

  void onEvent(std::function<void(const WiiEvent &)> eventListener);
  void sync(bool enable);
  void step();

  // True while an unpaired board is allowed to connect and complete PIN entry.
  bool pairingAllowed_() const { return static_cast<int64_t>(pairingAllowedUntil_ - millis()) > 0; }

  // Single unconditional HCI disconnect. The board is only treated as disconnected
  // once the controller confirms the link is gone (HCIDisconnected). Rejecting an
  // immediate re-page is handled by rejectBoardUntil_.
  void disconnect(uint16_t handle);

  // Forget the stored link keys so the affected boards have to pair again.
  bool remove_link_key(uint64_t bdaddr);
  int remove_all_link_keys();

 protected:
  void onACLChannelClosedUnexpectedly_(uint16_t handle, uint16_t psm);
  void handleBoardGone_(uint16_t handle);
};

}  // namespace esphome::wii_balance_board::detail
