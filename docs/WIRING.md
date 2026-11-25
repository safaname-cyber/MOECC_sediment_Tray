# Sediment Tray: wiring for FW 1.8.3 on the MOECC-2 board (ESP32-S3)

**Status of each pin**
- **Board-proven:** the same pin and function is already used by working MOECC-2 firmware: `motor_comm_test.c`, `rs485_test.c`, `main_eth.c`, the HX711 bench test, and the grabber.
- **Check:** used by the firmware, but its MOECC-2 connector hasn't been confirmed against the board schematic.

## 1. Pin map

| Function | ESP32-S3 GPIO | Direction | Status | Firmware name |
|---|---|---|---|---|
| Ethernet converter: ESP TX → converter RX | **17** | out | Board-proven | `PIN_UART1_TX` |
| Ethernet converter: ESP RX ← converter TX | **18** | in | Board-proven | `PIN_UART1_RX` |
| Servo RS-485: ESP TX → transceiver DI | **47** | out | Board-proven | `PIN_UART2_TX` |
| Servo RS-485: ESP RX ← transceiver RO | **13** | in | Board-proven | `PIN_UART2_RX` |
| Servo RS-485 transceiver SHDN (held HIGH) | **21** | out | Board-proven | `PIN_RS485_SHDN` |
| HX711 DOUT | **41** | in | Board-proven | `PIN_HX711_DOUT` |
| HX711 SCK | **42** | out | Board-proven | `PIN_HX711_SCK` |
| Limit switch OPEN | **6** | in, pull-up | Check: GPIO6 is the grabber's leak input | `PIN_LIMIT_OPEN` |
| Limit switch CLOSED | **7** | in, pull-up | Check: GPIO7 is the winch encoder B input | `PIN_LIMIT_CLOSED` |
| Solenoid lock (`LOCK` net → on-board Q2 AO3400A driver) | **10** | out | Board LOCK net per the MOECC-2 schematic. Lock-circuit issue fixed 2026-11-23. | `PIN_LOCK` |
| Status LED (optional) | 38 | out | Check | `PIN_STATUS_LED` |
| Fault LED (optional) | 39 | out | Check | `PIN_FAULT_LED` |
| Console / flashing | USB (UART0 43/44 or USB-JTAG) | — | Board-proven | — |

**Not used. Leave unconnected:** GPIO8 (old DE pin, `RS485_USE_DE 0`), GPIO2, GPIO16 (reserved), GPIO4 and GPIO5 (the old HX711 pins).

## 2. Connections

### 2.1 Ethernet (UART-to-Ethernet converter)
- GPIO17 → converter **RX**. GPIO18 ← converter **TX**. Connect GND to GND.
- Serial settings: **115200 baud, 8N1**, no flow control. 3.3 V logic.
- Converter mode: TCP server (or UDP) on its own IP and port. The PC client is `tools/sediment_client.py`.

### 2.2 Servo (Feetech RSBL45-24, RS-485)
- Use the board's on-board transceiver: GPIO47 → DI, GPIO13 ← RO. **SHDN (GPIO21) must be HIGH**; the firmware drives it high at boot.
- Transceiver **A → servo A, B → servo B**. If the servo doesn't answer, swap A and B.
- Servo power: **24 V** from the bus supply, rated at least 3 A. **Join the 24 V supply ground, board GND and transceiver GND.**
- Servo settings (set with FD): **ID 1, 115200 baud**. The firmware switches it into wheel (speed) mode at boot.

### 2.3 Limit switches (the only thing that ends an open or close move)
- Use a **normally-open** microswitch on each end. Wire COM → **GND** and NO → **GPIO6** (open end) or **GPIO7** (closed end).
- The internal pull-up is enabled. A pressed switch reads 0 V (LOW). The firmware debounces for 8 ms.
- For cables longer than about 30 cm, add 1 kΩ in series plus 100 nF from the pin to GND at the board.
- **Mount each switch so the tray presses it before any hard stop.** After a switch triggers, the motor needs a short distance to brake (speed `MOTION_SPEED` 2400, accel 50).

### 2.4 Solenoid lock (fail-safe: no power = locked). The driver is on the MOECC-2 board.
```
LOCK+ (+12 V) ──┬──────────────── Solenoid (+)
                │
           D9 SS34 (cathode → +12 V, anode → LOCK_GND)
                │
LOCK_GND ───────┴──────────────── Solenoid (−)
   │
   Q2 AO3400A drain (3);  source (2) → 12V_GND
GPIO10 (LOCK) ── R24 100 Ω ── Q2 gate (1);  gate ── R22 10 kΩ ── GND
```
- Solenoid between the **LOCK+** and **LOCK_GND** pins of the board's LOCK connector. **12 V coil.**
- GPIO10 **LOW = locked** (solenoid off). **HIGH = unlocked** (solenoid on).
- The 2026-11-20 lock-circuit reset issue was fixed on the board (2026-11-23). Lock re-enabled in FW 1.9.4.

### 2.5 Load cell (HX711)
- HX711 VCC → 3.3 V, GND → GND, **DOUT → GPIO41, SCK → GPIO42**.
- Load cell: red → E+, black → E−, green → A+, white → A−. If weight reads negative, swap A+ and A−; a negative scale is also accepted.

## 3. Firmware behaviour (FW 1.8.0)
- **`home`, `close`:** run towards CLOSED **until the GPIO7 switch is pressed**, with no time or travel limit. Then stop, lock, torque off.
- **`open`:** runs towards OPEN **until the GPIO6 switch is pressed**, then stops and holds.
- **`halt`:** stops a running move straight away. **`estop`:** latches a fault and turns torque off.
- **Faults that stop a move:**
  - stall: encoder shows no progress for 500 ms;
  - wrong direction: encoder moving against the command;
  - over-current, voltage or temperature;
  - lost host comms (production mode only).
- **If a switch is broken or missing:** the motor keeps turning until a stall, `halt` or `estop`. If the mechanism can't jam against a hard stop, set `MOTION_ENDSTOP_TIMEOUT_MS` (0 = off) as a backstop.

## 4. Bench checklist
1. Measure GPIO21 → 3.3 V (transceiver enabled), and GPIO6 and GPIO7 → 3.3 V with the switches released.
2. Boot log shows `motor response detected` and `"detected":true`.
3. In bench mode, `jog 300` turns the mechanism towards OPEN. If it faults with `wrong_dir`, flip `MOTION_ENC_SIGN`. If it turns the wrong physical way, flip `MOTION_OPEN_DIR`.
4. Send `open`: the motor keeps turning. Press the GPIO6 switch: it stops. Do the same for `close` / `home` with GPIO7.
5. During a move, send `halt`: the motor stops immediately.
