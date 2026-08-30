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
| VII | Board paged ESP, **crypto + open PSM 0x0013 directly** (18:08:32) | accept as slave (`role=0x01`); auth (stored key found); encrypt OK; ESP opens PSM `0x0013` directly (skipping `0x0011`) | `L2CAP_CON_RSP result=0x0004` (connection refused); disconnect `0x13` after ~1 s | As baseband **slave** the board **refuses PSM 0x0013 directly** (`result=0x0004`) exactly as it refused PSM 0x0011 in Attempt II. Definitively proves the board rejects *all* host-initiated L2CAP channels regardless of PSM. |

## 3. Proven constraints (rules this board obeyed in every test)

- R1. Baseband link succeeds only when: the board pages us, **or** the board is page-scanning (sync mode, ESP pages it).
- R2. When the board is pager (baseband master): it never opens L2CAP to us (I, III), refuses our L2CAP requests on both PSM 0x0011 and PSM 0x0013 (`0x0004`, II & VII), and hangs up on its own after ~1.0–1.1 s (`0x13`).
- R3. The board refuses to become baseband slave via role switch (`0x35`, IV).
  → **A board-initiated link can never yield a working L2CAP session on this stack (I + II + III + IV + VI + VII).**
- R4. ESP-paged (master) link works **only** while the board is in sync/discoverable mode (R5, R6 bound the rest).
- R5. Bound + idle board does not answer host pages (`0x08`, attempt V).
- R6. Bound + board-in-auto-reconnect does not answer host pages (`0x04`, attempt VI R2).
  → **R1+R5+R6: the board is page-scanning only while user-triggered sync mode is active.**

## 4. Do-not-repeat list (avoid these as "new ideas")

1. "Accept the page; the board will open L2CAP as master" → disproven (I, III).
2. "Accept the page; open PSM `0x0011`/`0x0013` as slave" → disproven (II: `0x0011` refused `0x0004`; VII: `0x0013` refused `0x0004`, then `0x13`).
3. "Accept with `role=0x00` (request master)" → disproven (IV; `0x35` refusal).
4. "Reject the page and immediately re-page to steal the link" → disproven (VI). Also note the controller self-collision (`0x0B`) if the create fires before the rejection's Connection Complete; that fix alone does **not** change the outcome (clean 5.12 s window, VI R2, was silent).
5. "ESP pages the bound board to reconnect it without user action" → disproven (V `0x08`, VI R2 `0x04`).
6. "Blame the stored/link key" for the L2CAP/role failures → **not a factor**: in-session reconnects used a RAM/stored key; auth+encryption *succeeded*; failure was always at L2CAP/role level. (The key-persistence bug is real but separate — §6.)

### Considered and deferred (do not run without new evidence)
- **Explicit `HCI Switch_Role` after accepting as slave:** sends the same LMP role-change PDU the board already refused at accept-time (`0x35`, IV). Not run; expected to fail identically.
- **Change the reject reason (e.g. `0x0D`) to keep the board paging longer:** speculative; does not address that the board never page-scans during reconnect (R6).

## 5. Conclusion (evidence-backed)

With a bound board, reconnect **without user action is not achievable** on this ESP32 classic-BT stack: the board initiates reconnect as baseband master (broken per R2/R3) and it will not answer our pages outside sync mode (R5/R6). The only supported paths are:

- **Pairing / re-sync** (sync button held, ESP page-scan) — works, this is the recovery path.
- **Within-session data flow** (ESP paged as master, then scheduled disconnect) — works.

#### Recommended workflow (supported recovery path)

Board needs a session again → user presses **SYNC** on the board → ESP (already paging/in pairing mode) connects as master → stored link key (once NVS fix with §6 is validated) → PIN-free auth in ~2 s. This is the intended UX as long as the board's firmware refuses every host-driven reconnect route (R2/R3/R5/R6).

## 6. Open item — link-key persistence (VALIDATED)

`saveLinkKey_` (bluetooth.cpp) saves the pairing's link key to NVS namespace `wii_bb`; `loadLinkKeys_` reads it at boot.
- Key name shortened to `"lk%012llX"` (14 chars, under 15-char limit).
- **Validation completed during Attempt VII (18:08:20):**
  - First pair: `NVS readback key=lk8C56C5CBCC4F err=ESP_OK size=16 match=1`
  - Disconnect / Reconnect attempt: `Link key found, replying for 4F CC CB C5 56 8C`
  - PIN-free authentication succeeded without re-entering PIN.

## Appendix — reference

- Board identifies as "Nintendo RVL-WBC-01" (remote name request).
- Board page cadence when we *accept*: ~5.8 s between attempts; session held ~1.0–1.1 s then `0x13`. After *reject*: single page, then silent ≥ 19 s.
- Key HCI codes seen: `0x04` page timeout, `0x08` connection timeout, `0x0B` command disallowed, `0x0F` connection rejected, `0x13` remote hangup, `0x35` LMP PDU not allowed, `0x0004` L2CAP connection refused.
- Commands: `& .venv\Scripts\esphome.exe compile|upload|logs wii_balance_board_esp32.yaml --device COM14` (from `C:\Temp\wii_balance_board`).

## Appendix — evidence index

Provenance legend: **disk** = forward-ref'd file under this repo; **chat** = exists only in the session conversation history (capture, not on disk).

| Claim | Capture | Provenance | What to look for |
|-------|---------|------------|------------------|
| Bound + idle board is not page-scanning (`0x08`) | 16:11 | *chat* | `Connection Complete status=0x08` (~20 s), zero Connection Request events |
| As slave the board refuses host-opened L2CAP (0x0011) | 16:25:37 | *chat* | `L2CAP_CON_RSP result=0x0004`, then `0x13` |
| As slave the board refuses host-opened L2CAP (0x0013 direct) | 18:08:33 | `ATTEMPTS.md` / *chat* | `L2CAP_CON_RSP result=0x0004`, then `0x13` |
| As master the board never opens L2CAP (no crypto) | 15:18 | *chat* | session `0x13` hangup ~1.1 s, zero L2CAP frames |
| As master the board never opens L2CAP (crypto) | 16:30:04 | *chat* | same, after successful auth+encrypt |
| Board refuses master/slave role switch | 16:33:37 / 16:33:43 | *chat* | `Role Changed status=0x35`, `new_role=0x01`, twice ~5.8 s apart |
| Reject+re-page fails; board silent after rejection | 17:00 | `LOGS_attempt_VI.txt` | round-1 `status=0x0B`, clean round-2 `status=0x00` → `failed status=0x04` after 5.12 s, then ≥ 19 s silence |
| Page-hold rhythm while accepting | 16:33 (dual) | *chat* | ~5.8 s page gap, ~1.0–1.1 s hold, `0x13`, repeated |

## Appendix — firmware build ↔ attempt mapping

| Attempt(s) | Build | config_hash / source |
|------------|-------|----------------------|
| I–III (accept only, no role request) | accept-flow build | earlier session compiles |
| IV (accept `role=0x00`, request master) | 16:31–16:32 build | instrumentation for Role Changed |
| V (page bound idle board) | reject+repage precursor build | 16:11 |
| VI + NVS instrumentation | 16:53 build (uploaded 16:54) | `config_hash=0xfdf0bbe6` |
| VII (accept slave + PSM 0x0013 direct) | 18:06 build | `config_hash=0xfdf0bbe6` |

Useful every time the goal of reconnect is revisited: identify the exact build behind any *new* capture before comparing it with these results.

## Appendix — HCI / decoding gotchas

- **`accepted` flag**: defined as `!connectRequests.contains(bdaddr)`. Consequence: the **Connection Complete of a *rejected incoming page* is logged as `accepted=false`** if any outbound `Create_Connection` was queued in between (seen in VI: the `0x0F` completion logged as "Connection to ... failed"). Do not interpret that line as a failed outbound connection by itself.
- **Timing**: a rejected page's Connection Complete arrives ~150 ms after its Connection Request and carries a real handle (e.g. `0x0081`). Firing `Create_Connection` in that window self-collides at the controller → `status=0x0B` (Command Disallowed). The minimum safe rearm delay is "after the rejection's Connection Complete," not "100 ms."
- **Reject reason** was `0x0F` (Connection Rejected Due To Unacceptable BD_ADDR) — the HCI primitive `Reject_Connection_Request` is all-or-nothing; different reason codes do not change page-scan behavior (see §4 "Considered and deferred").
- **Endianness**: BD_ADDR appears little-endian in HCI/ACL bytes on this stack — `4F CC CB C5 56 8C` (accepted byte-forward on wire). Logs print the human-oriented form.
- **PIN pairing**: replied with PIN method 1 (ESP's own MAC reversed); `Link Key Notification` reported `keyType=0x00`. Neither the PIN method nor keyType varied across attempts.