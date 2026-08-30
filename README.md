# Wii Balance Board ESPHome component

Use a Wii Balance Board as a smart scale in Home Assistant.

<img width="329" height="583" alt="image" src="https://github.com/user-attachments/assets/92ac038c-ad78-400b-9394-d109598193c5" />

More in depth documentation available [here](https://tightloop.io/homeassistant+balanceboard/index.html).

This fork is based on [gulrotkake's balance-board component](https://github.com/gulrotkake/esphome/tree/balance-board) and fixes **board-initiated reconnection**, which never worked upstream: after the first weighing, the ESP32 could only talk to the board again after a manual sync pairing. With this fork the board reconnects automatically whenever someone steps on it.

## How reconnection works in this fork

The RVL-WBC-01 pages the previously paired host whenever it is stepped on. The Bluetooth baseband pager becomes the link's initial master, but the board's firmware only proceeds to authentication and L2CAP once the *host* has taken the master role. Two things were required to make that happen on an ESP32:

1. **The btdm controller must be built for BR/EDR.** ESPHome's default sdkconfig leaves the controller at `BTDM_CTRL_MODE_BLE_ONLY` (with `BR_EDR_MAX_ACL_CONN_EFF=0`) even when `CONFIG_CLASSIC_BT_ENABLED` is set, because arduino-esp32's `btStart()` enables classic at runtime regardless. In that configuration the accept-time role switch fails (`Role Changed status=0x35`) and classic links behave erratically. The sample configuration below sets `CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY`, which compiles the controller for classic-only use and fixes the role switch.
2. **The host must become master and defer auth.** The component now accepts the board's page with `role=0x00`, sends `Write_Default_Link_Policy_Settings(0x0001)` during init so the controller is permitted to switch roles, and issues an explicit `Switch_Role` after the link is established. `Authentication_Requested` is deferred until the `Role Changed` event resolves. Once the host is master and the link is encrypted, the *board* opens L2CAP PSM 0x11/0x13 — the host no longer tries to open them itself on a reconnect.

Link keys are persisted in NVS (`lk<MAC-reversed>` keys in the `wbb` namespace), so re-authentication during reconnect does not require the sync-button PIN flow.

## Requirements

1. A balance board
2. A home assistant setup
3. An ESP32 device with support for BR/EDR.

## Sample Configuration

```
esp32:
  variant: esp32
  board: esp32dev # Replace with your esp32 board
  framework:
    type: arduino
    sdkconfig_options:
      CONFIG_BT_ENABLED: y
      CONFIG_BLUEDROID_ENABLED: y
      CONFIG_CLASSIC_BT_ENABLED: y
      # Required: build the controller for classic-only mode so the
      # accept-time role switch works (default BLE_ONLY breaks it).
      CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY: y

external_components:
  - source:
      type: local
      path: ./wii_balance_board_component
    components: [ wii_balance_board ]

wii_balance_board:
    id: board
    standard_deviation: 0.3

button:
  - platform: template
    name: "Start Sync"
    on_press:
      then:
        - lambda: 'id(board)->sync(true);'
```

## Logging

The component's raw HCI/ACL hex dumps (`HCI_EVT`, `ACL_RX_DATA`, ...) are logged at `VERBOSE`; connection lifecycle events are `INFO` and low-level detail is `DEBUG`. For troubleshooting a flaky link, set `logger: level: VERBOSE` (or `DEBUG`) to get the byte-level traces back.

## Contributions

- The original wiimote code (wiimote_bt.h, Wiimote.h and Wiimot.cpp) was from https://github.com/takeru/Wiimote, and was extended in https://github.com/gulrotkake/esp32_wiimote .
