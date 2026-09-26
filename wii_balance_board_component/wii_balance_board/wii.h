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
  std::optional<uint64_t> pendingReconnect;
  bool reconnecting{false};
  uint32_t rejectBoardPagesUntil{0};
  std::function<void(const WiiEvent &)> eventListener;

 public:
  Wii(Bluetooth *bluetooth);
  ~Wii();
  Wii(const Wii &) = delete;
  Wii &operator=(const Wii &) = delete;

  void onEvent(std::function<void(const WiiEvent &)> eventListener);
  void sync(bool enable);
  void step();

  void disconnect(uint16_t handle, uint16_t psm);
};

}  // namespace esphome::wii_balance_board::detail
