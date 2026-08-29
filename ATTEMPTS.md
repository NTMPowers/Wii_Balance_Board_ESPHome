# Wii Balance Board — Attempt Log & Failure Analysis

**Board:** Nintendo RVL-WBC-01, MAC `4F CC CB C5 56 8C`, CoD `0x042500`
**Host:** ESP32 (ESPHome component, ESP-IDF v5.5.5, arduino-esp32 3.3.11). HCI is driven directly via `btdm` / VHCI (e.g. `Bluetooth::Impl` in `bluetooth.cpp`).
**Goal:** an already-paired board reconnects (e.g. after stepping on it / after the ESP disconnects) *without* re-pairing and without user action.

> Purpose of this file: every idea already tried, and **exactly why it failed**, so we never re-run these. "Why it failed" is evidence-based; hypotheses that would duplicate a failed mechanism are explicitly rejected below.

---

## 1. What works (the only working configuration)

- ESP is baseband **master** (ESP pages the board), which only works while the board is **page-scanning = sync/discoverable mode** (sync button held during pairing / re-sync).
- Flow: ESP `Create_Connection` → board links → auth (`Link key found` from RAM or stored key) → encryption OK → ESP opens **PSM `0x0011` then `0x0013`** → calibration read → reporting mode `0x34` → valid sensor samples → component schedules disconnect (after valid sample, ≤ 15 s).

## 2. Attempts (chronological) and why each failed

Legend: "board paged ESP" = board is baseband **master**; "ESP paged board" = ESP is baseband **master**.

| # | Attempt | Config | Result | Why it failed (root cause) |
|---|---------|--------|--------|---------------------------|
| I | Board paged ESP, **no crypto**, wait (A, 15:18) | accept page; no auth/encrypt; wait for the board to open L2CAP | board opened **nothing**; disconnect reason `0x13` after ~1.1 s | As baseband master the board **never opens L2CAP** to the host. |
| II | Board paged ESP, **crypto + we open L2CAP** (B, 16:25:37) | accept; auth (RAM key found); encrypt OK; ESP opens PSM `0x0011` | `L2CAP_CON_RSP result=0x0004` (connection refused); disconnect `0x13` after ~1 s | As baseband **slave** the board **refuses L2CAP channels opened by the host**. |
| III | Board paged ESP, **crypto + wait** (C, 16:30:04) | accept; auth; encrypt; log "waiting for board to open L2CAP"; no opens | board opened **nothing**; disconnect `0x13` after ~1 s | Same as I. Also **refutes the xwiimote "PROTOCOL" model** ("on auto-reconnection the wiimote connects to the host") for this board. |
| IV | Board paged ESP, **request master role** (D, 16:33:37 and 16:33:43) | accept with `role=0x00` (become master); auth; encrypt; ESP opens PSM `0x0011` | `Role Changed status=0x35` (`LMP PDU not allowed`), `new_role=0x01` (still slave); then PSM `0x0011` refused `0x0004`; disconnect `0x13` after ~1 s. Happened twice, ~5.8 s apart. | Board **refuses the master/slave role switch at LMP level** (0x35) and, still master, refuses host-opened L2CAP. |
| V | ESP paged a **bound board in idle/standby** (16:11) | firmware was a reject+repage build; ESP queued `Create_Connection` to the already-paired board during its own sync scan | `Connection Complete status=0x08` (timeout) after ~20 s; **zero** Connection Request events arrived | A bound board in standby is **not page-scanning**. Page-scan only happens in sync/discoverable mode. NOTE: this was a page during idle, **not** a repage against a live board page — repage stayed untested until VI. |
| VI | **Reject + re-page** (during board auto-reconnect window, 17:00) | reject board page (reason `0x0F`); 100 ms later ESP `Create_Connection`; retry every 700 ms, up to 10 rounds | R1: `Create_Connection status=0x0B` (**Command Disallowed** — our controller was still completing the rejected page; its Connection Complete `0x0F` arrived ~150 ms later). R2 (~1.3 s after page): create accepted (`0x00`) → full 5.12 s page window → **`status=0x04` Page Timeout, no response**. Board paged once more total, then **silent for 19+ s** (compare sustained ~5.8 s cadence when we *accepted* its pages in II–IV). | Three independent causes: (a) rejecting + immediately `Create_Connection` **self-collides in our own controller** (R1 `0x0B` — avoidable by waiting for the rejection's Connection Complete); (b) a board in its auto-reconnect window **does not page-scan** — a clean 5.12 s window got zero answer; (c) rejection makes the board **back off / stop paging**. Refutes the ESP32Wiimote "always page, never accept" reconnect model for this board. |

## 3. Proven constraints (rules this board obeyed in every test)

- R1. Baseband link succeeds only when: the board pages us, **or** the board is page-scanning (sync mode, ESP pages it).
- R2. When the board is pager (baseband master): it never opens L2CAP to us, refuses our L2CAP requests (`0x0004`), and hangs up on its own after ~1.0–1.1 s (`0x13`).
- R3. The board refuses to become baseband slave via role switch (`0x35`).
  → **A board-initiated link can never yield a working L2CAP session on this stack (I + II + III + IV + VI).**
- R4. ESP-paged (master) link works **only** while the board is in sync/discoverable mode (R5, R6 bound the rest).
- R5. Bound + idle board does not answer host pages (`0x08`, attempt V).
- R6. Bound + board-in-auto-reconnect does not answer host pages (`0x04`, attempt VI R2).
  → **R1+R5+R6: the board is page-scanning only while user-triggered sync mode is active.**

## 4. Do-not-repeat list (avoid these as "new ideas")

1. "Accept the page; the board will open L2CAP as master" → disproven (I, III).
2. "Accept the page; open PSM `0x0011`/`0x0013` as slave" → disproven (II; `0x0004`, then `0x13`).
3. "Accept with `role=0x00` (request master)" → disproven (IV; `0x35` refusal).
4. "Reject the page and immediately re-page to steal the link" → disproven (VI). Also note the controller self-collision (`0x0B`) if the create fires before the rejection's Connection Complete; that fix alone does **not** change the outcome (clean 5.12 s window, VI R2, was silent).
5. "ESP pages the bound board to reconnect it without user action" → disproven (V `0x08`, VI R2 `0x04`).
6. "Blame the stored/link key" for the L2CAP/role failures → **not a factor**: in-session reconnects used a RAM key; auth+encryption *succeeded*; failure was always at L2CAP/role level. (The key-persistence bug is real but separate — §6.)

### Considered and deferred (do not run without new evidence)
- **Explicit `HCI Switch_Role` after accepting as slave:** sends the same LMP role-change PDU the board already refused at accept-time (`0x35`, IV). Not run; expected to fail identically.
- **Change the reject reason (e.g. `0x0D`) to keep the board paging longer:** speculative; does not address that the board never page-scans during reconnect (R6).

## 5. Conclusion (evidence-backed)

With a bound board, reconnect **without user action is not achievable** on this ESP32 classic-BT stack: the board initiates reconnect as baseband master (broken per R2/R3) and it will not answer our pages outside sync mode (R5/R6). The only supported paths are:

- **Pairing / re-sync** (sync button held, ESP page-scan) — works, this is the recovery path.
- **Within-session data flow** (ESP paged as master, then scheduled disconnect) — works.

## 6. Open item (not an attempt, separate bug) — link-key persistence

`saveLinkKey_` (bluetooth.cpp) saves the pairing's link key to NVS namespace `wii_bb`; `loadLinkKeys_` reads it at boot. Observed: after every successful pairing, the **next boot logs `Loaded 0 link keys`**. This is a **real bug** and blocks PIN-free *re-sync* (cross-boot auth), but is unrelated to attempts #2-#6 failures.

In-flight fix (as of 16:53 build, config_hash `0xfdf0bbe6`): key name shortened `"lk_%012llX"` (15 chars) → `"lk%012llX"` (14 chars); `nvs_commit` result checked; immediate `nvs_get_blob` read-back logged after save; loader iterates `NVS_TYPE_ANY` and logs every entry. Validation still pending: pair once → expect `NVS readback ... match=1` → reboot → expect `Loaded 1 link keys`. If read-back is `match=0`, the write (flash/partition) fails; if `match=1` but boot still loads 0, the read path fails.

## Appendix — reference

- Board identifies as "Nintendo RVL-WBC-01" (remote name request).
- Board page cadence when we *accept*: ~5.8 s between attempts; session held ~1.0–1.1 s then `0x13`. After *reject*: single page, then silent ≥ 19 s.
- Key HCI codes seen: `0x04` page timeout, `0x08` connection timeout, `0x0B` command disallowed, `0x0F` connection rejected, `0x13` remote hangup, `0x35` LMP PDU not allowed, `0x0004` L2CAP connection refused.
- Commands: `& .venv\Scripts\esphome.exe compile|upload|logs wii_balance_board_esp32.yaml --device COM14` (from `C:\Temp\wii_balance_board`).