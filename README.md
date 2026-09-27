# Wii Balance Board ESPHome component

Use a Wii Balance Board as a smart scale in Home Assistant.

<img width="329" height="583" alt="image" src="https://github.com/user-attachments/assets/92ac038c-ad78-400b-9394-d109598193c5" />

More in depth documentation on the Bluetooth protocol available [here](https://tightloop.io/homeassistant+balanceboard/index.html).

This fork is based on [gulrotkake's balance-board component](https://github.com/gulrotkake/esphome/tree/balance-board) and fixes **board-initiated reconnection**, which never worked upstream: after the first weighing, the ESP32 could only talk to the board again after a manual sync pairing. With this fork the board reconnects automatically whenever someone steps on it.

## How it works

### One weighing per connection

The board is battery powered and shuts itself down after a weighing, so there is no long-lived
link to hold. Each weighing is therefore a fresh connection:

1. You step on the board, which powers it and pages the paired ESP32.
2. The ESP32 accepts the page, authenticates, and the two negotiate L2CAP channels.
3. The board's own calibration is read out of its EEPROM.
4. The board is **zeroed** (see below) while nothing is on it.
5. `Ready to weigh` goes on, and only then is a weight measured.
6. The ESP32 drops the link and the board powers off.

A connection that produces no valid measurement is also torn down on a timer, so a session
can never hang indefinitely.

### Zeroing the board on every connection

The board keeps three calibration points per load cell — 0 kg, 17 kg and 34 kg — plus a
reference temperature, in its own EEPROM. A reading is produced by interpolating each cell
between those points and correcting for the difference between the current temperature and
the reference temperature.

The 0 kg point drifts, and it is re-derived by the board on every boot, so the ESP32
re-establishes it at the start of each connection:

1. The board's 0/17/34 kg points and reference temperature are read from EEPROM.
2. The reporting mode is set to 0x34, which is what makes the board stream weight reports.
3. The raw sensors are averaged over 2 seconds with nothing on the board.
4. The average is written back as the new 0 kg points, and the temperature read during
   that window is written back as the new reference temperature.
5. Both are read back and compared against what was written.

A reading therefore comes from the board's own temperature-compensated calibration, with the
zero fixed in the board's EEPROM.

Zeroing runs before you step on, because it needs an empty board and a measurement needs you
on it. `Ready to weigh` goes on once the write has been verified, and only then is a weight
measured.

The average is discarded and retried, up to three times, if a sensor moved more than 200
counts over the window or the board already reads over 5 kg. If no usable average is found,
`Ready to weigh` stays off and no weight is published for that session.

The block checksum at `0xA4003C` is left as the factory set it, because no CRC32 over the
calibration block reproduces the value stored there in either byte order, so there is no
correct value to write. The rest of the factory block is logged at `DEBUG` on every connection
so the original calibration can be restored by hand.

### Pairing

A stored link key is what marks a board as paired, and only a board with one is allowed to
connect. The board pages the ESP32, the ESP32 checks its NVS for a key for that address, and
the connection is refused if there isn't one. The refusal is logged:

```
[I] [wii_balance_board.component:058] Refused unpaired board 00224C56A440
```

Answering a PIN request is what performs pairing, so a PIN request is only answered inside the
pairing window opened by **Start sync**. That button allows an unpaired board to connect and
complete PIN entry for 60 seconds, then runs an inquiry to find it. Press it whenever a board
needs pairing again, including right after using one of the removal buttons. Both removal
buttons disconnect a connected board immediately rather than leaving the session up until it
times out.

Link keys are persisted in NVS (`lk<MAC-reversed>` keys in the `wii_bb` namespace), so a later
reconnect re-authenticates from the stored key.

### Reconnection

The RVL-WBC-01 pages the previously paired host whenever it is stepped on. The Bluetooth
baseband pager becomes the link's initial master, but the board's firmware only proceeds to
authentication and L2CAP once the *host* has taken the master role. Two things make that
happen on an ESP32:

1. **The btdm controller must be built for BR/EDR.** ESPHome's default sdkconfig leaves the controller at `BTDM_CTRL_MODE_BLE_ONLY` (with `BR_EDR_MAX_ACL_CONN_EFF=0`) even when `CONFIG_CLASSIC_BT_ENABLED` is set, because arduino-esp32's `btStart()` enables classic at runtime regardless. In that configuration the accept-time role switch fails (`Role Changed status=0x35`) and classic links behave erratically. The sample configuration below sets `CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY`, which compiles the controller for classic-only use and fixes the role switch.
2. **The host must become master and defer auth.** The component accepts the board's page with `role=0x00`, sends `Write_Default_Link_Policy_Settings(0x0001)` during init so the controller is permitted to switch roles, and issues an explicit `Switch_Role` after the link is established. `Authentication_Requested` is deferred until the `Role Changed` event resolves. Once the host is master and the link is encrypted, the *board* opens L2CAP PSM 0x11/0x13, so on a reconnect the host waits for the channels instead of initiating them.

If the board fires a role switch of its own at the same instant as ours, the two LMP
transactions collide and we end up slave anyway, which strands the session. That collision
is detected and the role switch is retried once the link is encrypted and the board's own
attempt has long finished, which cannot collide again.

The board also re-pages within a few seconds of a clean disconnect while someone is still
standing on it. Those pages are rejected for 10 seconds after each attempt, which stops the
board from immediately re-connecting before it has been stepped off.

## Requirements

1. A balance board
2. A home assistant setup
3. An ESP32 device with support for BR/EDR.

## Sample Configuration

```yaml
esphome:
  name: wii-balance-board
  friendly_name: "Wii Balance Board"

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

logger:
  level: INFO

external_components:
  - source:
      type: local
      path: ./wii_balance_board_component
    components: [ wii_balance_board ]

wii_balance_board:
  id: wbb
  standard_deviation: 0.3
  ready_to_step_on:
    name: "Ready to weigh"
  weight:
    name: "Weight"

button:
  - platform: template
    name: "Start sync"
    on_press:
      then:
        - lambda: 'id(wbb)->sync(true);'
  - platform: template
    name: "Remove board"
    on_press:
      then:
        - lambda: 'id(wbb)->remove_link_key();'
  - platform: template
    name: "Remove all boards"
    on_press:
      then:
        - lambda: 'id(wbb)->remove_all_link_keys();'
```

### Options

| Option | Default | Description |
| --- | --- | --- |
| `standard_deviation` | `0.4` | Maximum standard deviation, in kg, for a set of samples to count as a steady weight. Lower is stricter. |
| `syncing` | `Syncing` | Binary sensor, on while scanning for a board. |
| `ready_to_step_on` | `Ready to weigh` | Binary sensor, on once the board has been zeroed and it is safe to step on. |
| `weight` | `Weight` | Measured weight. |
| `temperature_sensor` | `Temperature` | The board's current temperature. |
| `reference_temperature_sensor` | `Reference temperature` | The temperature the board was zeroed at. |
| `battery_level` | `Battery level` | Battery percentage. |

## Using it

Step on the board and wait for `Ready to weigh` before putting your full weight on it. Each
weighing is one connection: the ESP32 zeroes the board, measures, then drops the link and the
board powers off.

## Logging

Normal use logs five lines per weighing: the connection, the zeroing, the readiness, the
measurement, and the disconnect. Failures — a board that could not be zeroed, a rejected
write, a link that dropped unexpectedly — are logged at `WARN` or above, and nothing else is.

The byte-level detail (raw HCI/ACL traffic, the EEPROM reads and writes, the factory
calibration block, the averaged zero and the write read-backs) is at `DEBUG` and `VERBOSE`.
Set `logger: level: VERBOSE` when troubleshooting a link or bringing the zeroing up on new
hardware, and expect a lot of output.

## Contributions

- The original wiimote code (wiimote_bt.h, Wiimote.h and Wiimot.cpp) was from https://github.com/takeru/Wiimote, and was extended in https://github.com/gulrotkake/esp32_wiimote .
- Zeroing follows the procedure in Nintendo's own balance board programming manual and the
  register map in [WiiBrew](https://wiibrew.org/wiki/Wii_Balance_Board).
