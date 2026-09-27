#pragma once
#include "bluetooth.h"
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

using WiiEvent =
    std::variant<BalanceBoardConnected, BalanceBoardDisconnected, BalanceBoardData, ScanStarted, ScanStopped>;

class Wii {
  struct BalanceBoard;
  Bluetooth *bluetooth;
  std::unordered_map<uint16_t, std::unique_ptr<BalanceBoard>> connectedBoards;
  std::unordered_map<uint16_t, uint64_t> handleToBdaddr;
  std::unordered_set<uint16_t> pendingPSM13;
  std::unordered_set<uint16_t> pendingEncryption;
  std::unordered_set<uint16_t> initiatorHandles;
  // Boards whose accept-time role switch lost its LMP race. The board only opens L2CAP
  // while it is the link slave, so an unresolved collision leaves it master and waiting
  // for channels we never open. Consumed at encryption time to force the role.
  std::unordered_set<uint64_t> needsRoleSwitch;
  std::optional<uint64_t> pendingReconnect;
  bool reconnecting{false};
  uint64_t rejectBoardUntil_{0};
  std::function<void(const WiiEvent &)> eventListener;

 public:
  Wii(Bluetooth *bluetooth);
  ~Wii();
  Wii(const Wii &) = delete;
  Wii &operator=(const Wii &) = delete;

  void onEvent(std::function<void(const WiiEvent &)> eventListener);
  void sync(bool enable);
  void step();

  // Directly tears down the Bluetooth link (HCI disconnect), exactly like
  // `bluetoothctl disconnect <address>`. This is deliberately a single,
  // unconditional command: no staged per-channel L2CAP handshake, no
  // retries. The board is only considered disconnected once the
  // controller confirms the link is actually gone (HCIDisconnected),
  // never earlier. A short post-disconnect reconnect cooldown is applied
  // separately (see rejectBoardUntil_) to stop the board's own immediate
  // re-page from being accepted.
  void disconnect(uint16_t handle);

 protected:
  void onACLChannelClosedUnexpectedly_(uint16_t handle, uint16_t psm);
  void handleBoardGone_(uint16_t handle);
};

}  // namespace esphome::wii_balance_board::detail
