# Technical details

This covers how the Bluetooth link and calibration actually work — useful for debugging,
porting, or understanding the log output at `DEBUG`/`VERBOSE`. For normal use, see
[README.md](README.md).

## Zeroing

The board keeps three calibration points per load cell — 0 kg, 17 kg and 34 kg — plus a
reference temperature, in its own EEPROM. A reading is produced by interpolating each
cell between those points and correcting for the difference between the current
temperature and the reference temperature.

The 0 kg point drifts, and it is re-derived by the board on every boot, so the ESP32
re-establishes it at the start of each connection:

1. The board's 0/17/34 kg points and reference temperature are read from EEPROM.
2. The reporting mode is set to `0x34`, which is what makes the board stream weight
   reports.
3. The raw sensors are averaged over 2 seconds with nothing on the board.
4. The average is written back as the new 0 kg points, and the temperature read during
   that window is written back as the new reference temperature.
5. Both are read back and compared against what was written.

A reading therefore comes from the board's own temperature-compensated calibration, with
the zero fixed in the board's EEPROM.

Zeroing runs before you step on, because it needs an empty board and a measurement needs
you on it. `Ready to weigh` goes on once the write has been verified, and only then is a
weight measured.

The average is discarded and retried, up to three times, if a sensor moved more than 200
counts over the window or the board already reads over 5 kg. If no usable average is
found, `Ready to weigh` stays off and no weight is published for that session.

The block checksum at `0xA4003C` is left as the factory set it, because no CRC32 over the
calibration block reproduces the value stored there in either byte order, so there is no
correct value to write. The rest of the factory block is logged at `DEBUG` on every
connection so the original calibration can be restored by hand.

## Pairing internals

A stored link key is what marks a board as paired, and only a board with one is allowed
to connect. The board pages the ESP32, the ESP32 checks its NVS for a key for that
address, and the connection is refused if there isn't one.

Link keys are persisted in NVS (`lk<MAC-reversed>` keys in the `wii_bb` namespace), so a
later reconnect re-authenticates from the stored key.

Answering a PIN request is what performs pairing, so a PIN request is only answered
inside the pairing window opened by **Start sync**. That button allows an unpaired board
to connect and complete PIN entry for 60 seconds, then runs an inquiry to find it. Both
removal buttons disconnect a connected board immediately rather than leaving the session
up until it times out.

## Reconnection protocol

The RVL-WBC-01 pages the previously paired host whenever it is stepped on. The Bluetooth
baseband pager becomes the link's initial master, but the board's firmware only proceeds
to authentication and L2CAP once the *host* has taken the master role. Two things make
that happen on an ESP32:

1. **The btdm controller must be built for BR/EDR.** ESPHome's default sdkconfig leaves
   the controller at `BTDM_CTRL_MODE_BLE_ONLY` (with `BR_EDR_MAX_ACL_CONN_EFF=0`) even
   when `CONFIG_CLASSIC_BT_ENABLED` is set, because arduino-esp32's `btStart()` enables
   classic at runtime regardless. In that configuration the accept-time role switch fails
   (`Role Changed status=0x35`) and classic links behave erratically. The sample
   configuration sets `CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY`, which compiles the controller
   for classic-only use and fixes the role switch.
2. **The host must become master and defer auth.** The component accepts the board's
   page with `role=0x00`, sends `Write_Default_Link_Policy_Settings(0x0001)` during init
   so the controller is permitted to switch roles, and issues an explicit `Switch_Role`
   after the link is established. `Authentication_Requested` is deferred until the
   `Role Changed` event resolves. Once the host is master and the link is encrypted, the
   *board* opens L2CAP PSM `0x11`/`0x13`, so on a reconnect the host waits for the
   channels instead of initiating them.

If the board fires a role switch of its own at the same instant as ours, the two LMP
transactions collide and we end up slave anyway, which strands the session. That
collision is detected, and the role switch is retried once the link is encrypted and the
board's own attempt has long finished, which cannot collide again.

The board also re-pages within a few seconds of a clean disconnect while someone is still
standing on it. Those pages are rejected for 10 seconds after each attempt, which stops
the board from immediately re-connecting before it has been stepped off.

## Contributions

- The original wiimote code (wiimote_bt.h, Wiimote.h and Wiimot.cpp) was from https://github.com/takeru/Wiimote, and was extended in https://github.com/gulrotkake/esp32_wiimote .
- Zeroing follows the procedure in Nintendo's own balance board programming manual and the register map in [WiiBrew](https://wiibrew.org/wiki/Wii_Balance_Board).
