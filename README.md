# Wii Balance Board ESPHome component

Use a Wii Balance Board as a smart scale.

## How it works

Each weighing is one fresh connection — the board is battery powered and shuts itself
down afterward, so there's no link to keep open between uses.

1. Step on the board. It powers on and connects to the ESP32.
2. The board re-zeros itself with nothing on it yet, using its own calibration.
3. `Ready to weigh` turns on.
4. Step on and your weight is measured.
5. The board disconnects and powers off.

If a connection doesn't produce a valid measurement, it's automatically dropped after a
short timeout, so a session can never hang.

For how the Bluetooth link and calibration actually work under the hood, see
[TECHNICAL.md](TECHNICAL.md).

## Pairing

A board must be paired before it will connect.

1. Press **Start sync**. This opens a 60-second pairing window.
2. Step on the new board within that window and complete the PIN prompt.

Once paired, the board reconnects automatically every time you press the A-button.

- **Remove board** / **Remove all boards** forget the stored pairing(s).

## Requirements

1. A balance board
2. An ESP32 device with support for BR/EDR (classic) Bluetooth

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
      # Required — see TECHNICAL.md for why.
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

Two complete example configs, both set up for a WEMOS LOLIN32 Lite:

- [`wii_balance_board_esp32.yaml`](wii_balance_board_esp32.yaml) — full working config.
- [`wii_balance_board_esp32_pushover.yaml`](wii_balance_board_esp32_pushover.yaml) — the same, plus sends each completed measurement to Pushover.

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
| `led_pin` | none | GPIO with an LED that is lit while the board is ready to weigh. |
| `led_inverted` | `false` | Set to `true` for an LED that lights on a low output, such as the LOLIN32 Lite's on-board LED on GPIO 22. |
| `on_measurement` | none | Automation that runs whenever a weighing is accepted, with the measured weight as `weight` in kg. |

## Reacting to a weighing

`on_measurement` fires once per accepted weighing, so it can drive any automation. The
measured weight in kg is available as `weight`.

```yaml
wii_balance_board:
  id: wbb
  battery_level:
    id: wbb_battery
  on_measurement:
    then:
      - logger.log:
          format: "Weighed %.2f kg on %d%% battery"
          args: [ 'weight', '(int) lroundf(id(wbb_battery).state)' ]
```

Give a sensor an `id:` to read its value from a lambda, as `battery_level` does above.

## Using it

Connect the board and wait for `Ready to weigh` before stepping on it.
Each weighing is one connection: the ESP32 zeroes the board, measures, then drops the
link and the board powers off.

## Contributions

- The original wiimote code (wiimote_bt.h, Wiimote.h and Wiimot.cpp) was from https://github.com/takeru/Wiimote, and was extended in https://github.com/gulrotkake/esp32_wiimote .
- Zeroing follows the procedure in Nintendo's own balance board programming manual and the register map in [WiiBrew](https://wiibrew.org/wiki/Wii_Balance_Board).
