/*  ============================================================================
 *   sediment.c  —  EyeROV Sediment Tray Controller  ·  FW v1.3.9
 *
 *   Target        :  ESP32-S3-WROOM-1
 *   Framework     :  ESP-IDF (FreeRTOS)
 *   Program       :  MoECC QRDI
 *   Hardware      :  Feetech RSBL45-24 servo (RS-485)  ·  HX711 load cell
 *                    2 limit switches  ·  solenoid lock (fail-safe)
 *   Reference ICD :  v3.1.8  (sediment_tray_icd_v3_1_8.drawio)
 *   Reference flow:  v3.1.8  (sediment_tray_flowchart_v3_1_8_complete.drawio)
 *   Authority     :  EyeROV SOP for Embedded Design v1.0  +  Design Review Checklist
 *
 *   Architecture  :  4 tasks  +  Option B servo bus ownership (queue-based)
 *                    safety_task   P4 core0 1ms    WDT · limits · comms
 *                    status_task   P3 core0 100ms  SOLE UART2 TALKER:
 *                                                  motor telemetry + fault + hb + queue drain
 *                    json_cmd_task P2 core1 ev     UART1 I/O + 22 cmds inline
 *                    weight_task   P1 core1 20ms   HX711 + rolling avg + OVERWEIGHT
 *
 *   E-stop        :  COMMAND ONLY  —  no hardware pin  (GPIO 16 is reserved)
 *   Close flow    :  skips WEIGHING mode  —  instant weight after lock engages
 *   Capacity chk  :  open uses rolling 20 ms average, no stability required
 *   Fault model   :  latched  —  servo_torque(false) re-asserted every 100 ms
 *                    until reset_fault clears (if condition is gone)
 *
 *   JSON model    :  grabber-style, ZERO heap allocation, ZERO recursion
 *                    - Parsers: parse_json_cmd + parse_json_int_field (string search)
 *                    - Emitter: uart1_send_json(fmt, ...) → vsnprintf into stack buf
 *                    - Buffer:  JSON_LINE_BUF 1024 bytes on emission stack
 *                    - Truncation detected and logged, line dropped on overflow
 *                    - No cJSON, no external JSON library
 *
 *   SOP compliance notes:
 *     §4   real-time: WDT on safety/status/json_cmd, documented periods
 *     §6   comms: parser rejects malformed input defensively.
 *               NOTE (v1.3.2): 3-s host comms watchdog REMOVED by design —
 *               host silence does NOT trigger a fault. SOP §14 deviation.
 *     §7   control: endstop-wait with timeout, saturation guards on servo regs
 *     §8   coding: NO dynamic allocation anywhere, explicit widths, NO recursion,
 *                  default-case switches, _Static_assert on scaling factors
 *     §9   concurrency: all shared globals accessed under mutex or marked volatile,
 *                       servo bus single-owner (Option B — no mutex needed)
 *     §14  safe states on: motor runaway, sensor fail, brownout, estop
 *               (comms-loss safe-state REMOVED by design — see §6 note above)
 *     §16  version string + build date + boot_count + reset_cause in boot event
 *  ========================================================================= */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>   /* abs() */
#include <math.h>
#include <inttypes.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"

#include "nvs_flash.h"
#include "nvs.h"

/*  ============================================================================
 *   SECTION 1  ·  VERSION / BUILD
 *  ========================================================================= */

#define FW_VERSION_STR            "1.3.9"
#define FW_BUILD_DATE             __DATE__ " " __TIME__
#ifndef FW_GIT_HASH
#define FW_GIT_HASH               "release"
#endif

static const char *TAG            = "tray";

/*  ============================================================================
 *   SECTION 2  ·  PIN MAP   (ESP32-S3-WROOM-1)
 *  ========================================================================= */

#define PIN_HX711_DOUT            GPIO_NUM_4
#define PIN_HX711_SCK             GPIO_NUM_5
#define PIN_LIMIT_OPEN            GPIO_NUM_6
#define PIN_LIMIT_CLOSED          GPIO_NUM_7
#define PIN_RS485_DE              GPIO_NUM_8
#define PIN_LOCK                  GPIO_NUM_10
/*                                GPIO_NUM_16  — RESERVED (was ESTOP, removed v1.3.0) */
#define PIN_UART1_TX              GPIO_NUM_17
#define PIN_UART1_RX              GPIO_NUM_18
#define PIN_UART2_TX              GPIO_NUM_21
#define PIN_UART2_RX              GPIO_NUM_47
#define PIN_STATUS_LED            GPIO_NUM_38
#define PIN_FAULT_LED             GPIO_NUM_39

#define UART_JSON                 UART_NUM_1
#define UART_SERVO                UART_NUM_2
#define UART_CONSOLE              UART_NUM_0   /* native USB — idf.py monitor */

#define LOCK_ENGAGED_LEVEL        0   /* GPIO LOW  = solenoid OFF = LOCKED */
#define LOCK_DISENGAGED_LEVEL     1   /* GPIO HIGH = solenoid ON  = UNLOCKED */

/*  ============================================================================
 *   SECTION 3  ·  TIMING / LIMITS / THRESHOLDS
 *  ========================================================================= */

/* --- Task periods ------------------------------------------------------- */
#define SAFETY_PERIOD_MS          1
#define STATUS_PERIOD_MS          100
#define WEIGHT_PERIOD_MS          20

/* --- Watchdog / comms --------------------------------------------------- */
#define TASK_WDT_TIMEOUT_MS       200
/* COMMS_TIMEOUT_MS REMOVED (v1.3.2) — comms watchdog disabled by design.
 * Host silence does NOT trigger a fault in this firmware. g_last_rx_ms is
 * still tracked and visible via diag for host-side debugging only.         */
#define BOOT_GRACE_MS             500       /* skip limit/motor supervise during startup */

/* --- Motion ------------------------------------------------------------- */
#define ENDSTOP_TIMEOUT_MS        5000
#define LOCK_SETTLE_MS            500       /* solenoid pull-in / release */
#define SERVO_REBOOT_WAIT_MS      500
#define HOMING_STALL_MS           2000      /* position unchanged in HOMING */
#define MOTION_ACCEL              50        /* 0x29 acceleration */
#define MOTION_PWM_OPEN_SPEED     1000      /* 0x2C-0x2D PWM / open-loop speed */
#define MOTION_SPEED              2400      /* 0x2E-0x2F running speed (0 at power-up = no motion) */

/* --- Heartbeat / supervision -------------------------------------------- */
#define HEARTBEAT_PERIOD_MS       5000      /* BENCH ONLY — revert to 1000 before production flash */
#define SERVO_STATUS_POLL_MS      1000      /* 1 Hz slow poll of 0x41 */

/* --- Motor thresholds  (RSBL45-24 datasheet-derived) -------------------- */
#define MAX_CURRENT_MA            2500      /* locked-rotor 2300 mA + margin */
#define MAX_VOLTAGE_DV            250       /* 25.0 V in 0.1 V units */
#define MIN_VOLTAGE_DV            90        /*  9.0 V */
#define MAX_TEMP_C                65        /* datasheet 70 °C − 5 °C margin */
#define CURRENT_LSB_UA            6500      /* 6.5 mA per LSB on 0x45 */
#define CURRENT_SIGN_BIT          0x8000    /* bit15 = direction */
#define CURRENT_MAG_MASK          0x7FFF
#define CURRENT_SAT_COUNTS        2047      /* immediate OVERCURRENT trip */
#define VOLTAGE_SAT_COUNTS        250       /* immediate OVERVOLTAGE trip (≥ 25.0 V raw) */
#define FAULT_TRIP_COUNT          2         /* consecutive samples required */
#define BAD_READ_LIMIT            10        /* → MOTOR_SENSOR_INVALID (motion modes only) */
#define BAD_READ_WARN_PERIOD_MS   5000      /* static-mode bad_reads warn rate limit */

#define WARN_PCT                  80
#define WARN_HYST_PCT             72
#define WARN_CURRENT_MA           ((MAX_CURRENT_MA * WARN_PCT) / 100)
#define WARN_CURRENT_HYST_MA      ((MAX_CURRENT_MA * WARN_HYST_PCT) / 100)
#define WARN_TEMP_C               ((MAX_TEMP_C    * WARN_PCT) / 100)
#define WARN_TEMP_HYST_C          ((MAX_TEMP_C    * WARN_HYST_PCT) / 100)

#define PEAK_TRACK_START_MA       50
#define PEAK_TRACK_END_MA         50

/* --- Weight thresholds -------------------------------------------------- */
#define MAX_WEIGHT_G              1500.0f
#define WEIGHT_STABLE_WIN_MS      500
#define WEIGHT_STABLE_EPSILON_G   2.0f
#define HX711_RING_SIZE           16
#define HX711_TARE_SAMPLES        10
#define HX711_READ_TIMEOUT_US     100000

/* --- Limit switch debouncing -------------------------------------------- */
#define LIMIT_DEBOUNCE_SAMPLES    8

/* --- Servo mechanical range (bench-tune) -------------------------------- */
#define SERVO_POS_CLOSED          100
#define SERVO_POS_OPEN            3900
#define SERVO_ID                  0x01

/* --- Load cell NVS defaults --------------------------------------------- */
#define LC_DEFAULT_OFFSET         0
#define LC_DEFAULT_SCALE_CPG      100

/* --- JSON I/O ----------------------------------------------------------- */
#define JSON_LINE_MAX_IN          512       /* longest incoming line we accept */
#define JSON_LINE_BUF             1024      /* vsnprintf stack buffer per emission */
#define CONSOLE_LINE_MAX_IN       128       /* longest short-cmd line on UART0  */
#define FAULT_SRC_LEN             16

_Static_assert(JSON_LINE_BUF >= 512,
    "JSON_LINE_BUF must fit the longest emission (diag response ~700 B)");

/*  ============================================================================
 *   SECTION 4  ·  SERVO PROTOCOL   (Feetech SCS/STS, 115200 baud)
 *  ========================================================================= */

#define SREG_RESP_LEVEL           0x08      /* 0 = reply to READ/PING only, 1 = reply to all */
#define SREG_LOCK_FLAG            0x37
#define SREG_MAX_TEMP_LIMIT       0x0D
#define SREG_PROTECT_CURRENT      0x1C
#define SREG_UNLOADING_COND       0x13
#define SREG_OVERCURRENT_TIME     0x26
#define SREG_MODE                 0x21
#define SREG_TORQUE_ENABLE        0x28
#define SREG_ACCEL                0x29
#define SREG_GOAL_POS             0x2A
#define SREG_PRESENT_POS          0x38
#define SREG_PRESENT_VOLT         0x3E
#define SREG_PRESENT_TEMP         0x3F
#define SREG_SERVO_STATUS         0x41
#define SREG_PRESENT_CURRENT      0x45

#define SINST_PING                0x01
#define SINST_READ                0x02
#define SINST_WRITE               0x03
#define SINST_REBOOT              0x08

#define UC_VOLTAGE                (1 << 0)
#define UC_SENSOR                 (1 << 1)
#define UC_TEMPERATURE            (1 << 2)
#define UC_CURRENT                (1 << 5)
#define UC_OVERLOAD               (1 << 7)

#define SS_VOLTAGE                (1 << 0)
#define SS_SENSOR                 (1 << 1)
#define SS_TEMPERATURE            (1 << 2)
#define SS_CURRENT                (1 << 5)
#define SS_OVERLOAD               (1 << 7)

#define SENTINEL_WORD             0x7FFF
#define SENTINEL_BYTE             0xFF

#define MA_TO_COUNTS(ma)          ((uint16_t)(((int32_t)(ma) * 1000) / CURRENT_LSB_UA))

/*  ============================================================================
 *   SECTION 5  ·  TYPES
 *  ========================================================================= */

typedef enum {
    MODE_BOOT = 0,
    MODE_SELF_TEST,
    MODE_HOMING,
    MODE_IDLE,
    MODE_UNLOCKING,
    MODE_OPENING,
    MODE_OPEN,
    MODE_CLOSING,
    MODE_LOCKING,
    MODE_CLOSED_LOCKED,
    MODE_FAULT,
    MODE_COUNT_,
} tray_mode_t;

static const char *const MODE_NAME[MODE_COUNT_] = {
    [MODE_BOOT]           = "boot",
    [MODE_SELF_TEST]      = "self_test",
    [MODE_HOMING]         = "homing",
    [MODE_IDLE]           = "idle",
    [MODE_UNLOCKING]      = "unlocking",
    [MODE_OPENING]        = "opening",
    [MODE_OPEN]           = "open",
    [MODE_CLOSING]        = "closing",
    [MODE_LOCKING]        = "locking",
    [MODE_CLOSED_LOCKED]  = "closed_locked",
    [MODE_FAULT]          = "fault",
};

typedef enum {
    FAULT_NONE = 0,
    FAULT_OVERCURRENT,
    FAULT_OVERVOLTAGE,
    FAULT_UNDERVOLTAGE,
    FAULT_OVERTEMP,
    FAULT_MOTOR_SENSOR_INVALID,
    FAULT_OVERWEIGHT,
    FAULT_SENSOR_NAN,
    FAULT_MOTION_STALL,
    FAULT_HOMING_FAILED,
    FAULT_ENDSTOP_TIMEOUT,
    FAULT_BUS_BUSY,
    FAULT_COMMS_LOST,
    FAULT_ESTOP,
    FAULT_NVS_FAILED,
    FAULT_BROWNOUT,
    FAULT_WDT_RESET,
    FAULT_COUNT_,
} fault_name_t;

static const char *const FAULT_NAME[FAULT_COUNT_] = {
    [FAULT_NONE]                 = "none",
    [FAULT_OVERCURRENT]          = "OVERCURRENT",
    [FAULT_OVERVOLTAGE]          = "OVERVOLTAGE",
    [FAULT_UNDERVOLTAGE]         = "UNDERVOLTAGE",
    [FAULT_OVERTEMP]             = "OVERTEMP",
    [FAULT_MOTOR_SENSOR_INVALID] = "MOTOR_SENSOR_INVALID",
    [FAULT_OVERWEIGHT]           = "OVERWEIGHT",
    [FAULT_SENSOR_NAN]           = "SENSOR_NAN",
    [FAULT_MOTION_STALL]         = "MOTION_STALL",
    [FAULT_HOMING_FAILED]        = "HOMING_FAILED",
    [FAULT_ENDSTOP_TIMEOUT]      = "ENDSTOP_TIMEOUT",
    [FAULT_BUS_BUSY]             = "BUS_BUSY",
    [FAULT_COMMS_LOST]           = "COMMS_LOST",
    [FAULT_ESTOP]                = "ESTOP",
    [FAULT_NVS_FAILED]           = "NVS_FAILED",
    [FAULT_BROWNOUT]             = "BROWNOUT",
    [FAULT_WDT_RESET]            = "WDT_RESET",
};

typedef enum {
    SRV_CMD_GOAL = 0,
    SRV_CMD_TORQUE_OFF,
    SRV_CMD_TORQUE_ON,
    SRV_CMD_REBOOT,
} srv_cmd_type_t;

typedef struct {
    srv_cmd_type_t  type;
    uint16_t        goal_pos;
    uint32_t        origin_seq;
} srv_cmd_t;

#define SERVO_CMD_Q_DEPTH         4

/*  ============================================================================
 *   SECTION 6  ·  STATIC ASSERTIONS  (compile-time scaling safety — SOP §8.1)
 *  ========================================================================= */

_Static_assert(MA_TO_COUNTS(MAX_CURRENT_MA) < 500,
    "In-servo PROTECT_CURRENT exceeds datasheet-stated max register value (500).");
_Static_assert(MAX_VOLTAGE_DV <= 255,
    "MAX_VOLTAGE_DV must fit in one byte.");
_Static_assert(MIN_VOLTAGE_DV <= MAX_VOLTAGE_DV,
    "MIN_VOLTAGE_DV must be below MAX_VOLTAGE_DV.");
_Static_assert(MAX_TEMP_C < 128,
    "MAX_TEMP_C must fit in signed byte.");
_Static_assert(CURRENT_SAT_COUNTS <= 0x7FFF,
    "CURRENT_SAT_COUNTS must fit magnitude field (15 bits).");
_Static_assert(HX711_RING_SIZE > 0 && ((HX711_RING_SIZE & (HX711_RING_SIZE - 1)) == 0),
    "HX711_RING_SIZE must be a positive power of two.");
_Static_assert(SERVO_POS_CLOSED >= 0 && SERVO_POS_OPEN <= 4095,
    "Servo goal positions must be within 12-bit encoder range.");
_Static_assert(JSON_LINE_MAX_IN >= 128,
    "JSON_LINE_MAX_IN must fit the longest expected command line.");

/*  ============================================================================
 *   SECTION 7  ·  GLOBALS   (shared state — see SOP §9.2 / Checklist D)
 *  ========================================================================= */

/* --- Mode / fault ------------------------------------------------------- */
static volatile tray_mode_t     g_mode              = MODE_BOOT;
static volatile bool            g_fault_latched     = false;
static volatile fault_name_t    g_fault             = FAULT_NONE;
static char                     g_fault_src[FAULT_SRC_LEN] = "";
static volatile int32_t         g_fault_val         = 0;
static volatile uint32_t        g_fault_seq         = 0;
static volatile bool            g_homed             = false;

/* --- Load cell shared state (g_lc_mutex-protected) ---------------------- */
static SemaphoreHandle_t        g_lc_mutex          = NULL;
static int32_t                  g_lc_ring[HX711_RING_SIZE];
static uint8_t                  g_lc_ring_idx       = 0;
static uint8_t                  g_lc_ring_filled    = 0;
static int32_t                  g_lc_offset         = LC_DEFAULT_OFFSET;
static int32_t                  g_lc_scale_cpg      = LC_DEFAULT_SCALE_CPG;
static int32_t                  g_lc_last_raw       = 0;
static int32_t                  g_lc_last_avg       = 0;
static volatile float           g_weight_g          = 0.0f;
static volatile bool            g_weight_ok         = true;
static volatile bool            g_weight_stable     = false;
static uint32_t                 g_weight_stable_since_ms = 0;

/* --- Motor telemetry cache (status_task writes; others read) ------------ */
static volatile int32_t         g_motor_current_ma  = 0;
static volatile int32_t         g_motor_voltage_dv  = 0;
static volatile int32_t         g_motor_temp_c      = 0;
static volatile int32_t         g_motor_pos         = -1;
static volatile uint8_t         g_motor_servo_status = 0;
static volatile bool            g_motor_ok          = false;
static volatile uint32_t        g_motor_last_read_ms = 0;
static volatile uint8_t         g_motor_bad_reads_consec = 0;
static int32_t                  g_motor_peak_ma     = 0;
static bool                     g_motor_tracking_peak = false;
static bool                     g_warn_current_latched = false;
static bool                     g_warn_temp_latched = false;
static uint32_t                 g_motor_supervision_suspended_until_ms = 0;

/* --- Fault trip counters ------------------------------------------------ */
static uint8_t                  g_trip_overcurrent  = 0;
static uint8_t                  g_trip_overvoltage  = 0;
static uint8_t                  g_trip_undervoltage = 0;
static uint8_t                  g_trip_overtemp     = 0;

/* --- Comms watchdog ----------------------------------------------------- */
static volatile uint32_t        g_last_rx_ms        = 0;

/* --- Limit switches (safety_task writes; others read) ------------------- */
static volatile bool            g_lim_open          = false;
static volatile bool            g_lim_closed        = false;
static uint8_t                  g_lim_open_dbnc     = 0;
static uint8_t                  g_lim_closed_dbnc   = 0;
static bool                     g_lim_open_prev     = false;
static bool                     g_lim_closed_prev   = false;

/* --- Boot / reset info -------------------------------------------------- */
static uint32_t                 g_boot_count        = 0;
static esp_reset_reason_t       g_reset_cause       = ESP_RST_UNKNOWN;

/* --- Queues ------------------------------------------------------------- */
static QueueHandle_t            g_servo_cmd_q       = NULL;

/* --- NVS handle --------------------------------------------------------- */
static nvs_handle_t             g_nvs_h             = 0;
static bool                     g_motor_bus_ready   = false;

/* --- Comms stats -------------------------------------------------------- */
static uint32_t                 g_parse_err_count   = 0;
static uint32_t                 g_rx_line_count     = 0;
static uint32_t                 g_tx_line_count     = 0;
static uint32_t                 g_tx_trunc_count    = 0;

/* --- Console (UART0) ---------------------------------------------------- */
static volatile bool            g_console_ready     = false;
static volatile bool            g_usbjtag_ready     = false;   /* native USB (/dev/ttyACM0) */

/*  ============================================================================
 *   SECTION 7.5  ·  JSON HELPERS  (grabber v1.29.0 pattern — ZERO heap, ZERO
 *                   recursion. See send_boot_json / parse_json_cmd / parse_json_int_field
 *                   in grabber_v1_29_0.c for the lineage.)
 *
 *   RX (parsing):
 *     parse_json_cmd       — extract "cmd" field value into caller buffer.
 *     parse_json_int_field — extract any integer field by key name.
 *     parse_json_has_key   — true if key is present (used for optional seq).
 *     Both parsers scan the raw NUL-terminated line in place: no allocation,
 *     no tokenizer, no tree, no recursion. Malformed input is rejected
 *     (SOP §8.2 — defensive).
 *
 *   TX (emission):
 *     uart1_send_json(fmt, ...)  — vsnprintf into a 1 KB stack buffer, append
 *     "\r\n", single uart_write_bytes. Truncation detected, logged, dropped
 *     (never silently truncated frame). Caller is responsible for the content
 *     of fmt: JSON is produced by printf format strings. Values that may
 *     contain JSON-reserved characters (quote, backslash) must come from
 *     trusted sources — all host-facing strings here are compile-time
 *     literals (mode names, fault names, cmd names, reason codes) or
 *     fixed-charset NVS/version strings. No user-supplied strings are echoed.
 *  ========================================================================= */

/* ---- Parser: extract "cmd" field value --------------------------------- */
static bool parse_json_cmd(const char *raw, char *cmd_out, size_t cmd_out_len) {
    if (raw == NULL || cmd_out == NULL || cmd_out_len == 0) return false;
    cmd_out[0] = '\0';

    const char *key = strstr(raw, "\"cmd\"");
    if (key == NULL) return false;

    const char *colon = strchr(key + 5, ':');
    if (colon == NULL) return false;

    const char *q1 = strchr(colon + 1, '"');
    if (q1 == NULL) return false;

    const char *q2 = strchr(q1 + 1, '"');
    if (q2 == NULL) return false;

    size_t len = (size_t)(q2 - q1 - 1);
    if (len == 0 || len >= cmd_out_len) return false;

    memcpy(cmd_out, q1 + 1, len);
    cmd_out[len] = '\0';
    return true;
}

/* ---- Parser: extract integer value for a given key --------------------- */
static bool parse_json_int_field(const char *raw, const char *key, int32_t *out_value) {
    if (raw == NULL || key == NULL || out_value == NULL) return false;

    char key_pattern[40];
    int klen = snprintf(key_pattern, sizeof(key_pattern), "\"%s\"", key);
    if (klen <= 0 || klen >= (int)sizeof(key_pattern)) return false;

    const char *key_pos = strstr(raw, key_pattern);
    if (key_pos == NULL) return false;

    const char *colon = strchr(key_pos + klen, ':');
    if (colon == NULL) return false;

    const char *p = colon + 1;
    while (*p == ' ' || *p == '\t') p++;

    bool neg = false;
    if (*p == '-') { neg = true; p++; }
    if (*p < '0' || *p > '9') return false;

    long value = 0;
    int  digits = 0;
    while (*p >= '0' && *p <= '9') {
        value = value * 10 + (*p - '0');
        p++; digits++;
        if (digits > 10) return false;    /* reject absurd length */
    }
    if (neg) value = -value;

    *out_value = (int32_t)value;
    return true;
}

/* ---- Parser: cheap presence test for an optional key ------------------- */
static bool parse_json_has_key(const char *raw, const char *key) {
    if (raw == NULL || key == NULL) return false;
    char pat[40];
    int klen = snprintf(pat, sizeof(pat), "\"%s\"", key);
    if (klen <= 0 || klen >= (int)sizeof(pat)) return false;
    return strstr(raw, pat) != NULL;
}

/* ---- Emitter: static stack buffer + vsnprintf -------------------------- */
static void uart1_send_json(const char *fmt, ...) {
    if (fmt == NULL) return;
    char buf[JSON_LINE_BUF];
    va_list args;
    va_start(args, fmt);
    int32_t len = vsnprintf(buf, sizeof(buf) - 2, fmt, args);
    va_end(args);
    if (len <= 0 || len >= (int32_t)(sizeof(buf) - 2)) {
        g_tx_trunc_count++;
        ESP_LOGE(TAG, "uart1_send_json truncation (len=%ld)", (long)len);
        return;
    }
    buf[len]     = '\r';
    buf[len + 1] = '\n';
    uart_write_bytes(UART_JSON, buf, (size_t)(len + 2));
    /* Mirror every JSON frame to whichever console path(s) installed so
     * the user sees responses in idf.py monitor regardless of which USB
     * port on the devkit they plugged into.                              */
    if (g_console_ready) {
        uart_write_bytes(UART_CONSOLE, buf, (size_t)(len + 2));
    }
    if (g_usbjtag_ready) {
        usb_serial_jtag_write_bytes(buf, (size_t)(len + 2), pdMS_TO_TICKS(20));
    }
    g_tx_line_count++;
}

/* --- Convenience helpers for the standard trailer ----------------------- *
 * Every response and event carries ,"mode":"<...>","t":<ms>.
 * These macros are inserted at the end of every emit format string so the
 * emit call site reads clearly.                                             */
#define TRAILER_FMT       ",\"mode\":\"%s\",\"t\":%u"
#define TRAILER_ARGS      mode_name_str(), (unsigned)ms_now()

static uint32_t ms_now(void);     /* fwd — defined in §9 */
static const char *mode_name_str(void);

/*  ============================================================================
 *   SECTION 8  ·  FORWARD DECLARATIONS
 *  ========================================================================= */

/* NVS */
static esp_err_t nvs_restore_calibration(void);
static esp_err_t nvs_save_tare(int32_t offset);
static esp_err_t nvs_save_scale(int32_t scale_cpg);
static void      nvs_update_boot_count(void);

/* GPIO + lock */
static void gpio_setup(void);
static void lock_engage(void);
static void lock_disengage(void);

/* HX711 */
static bool    hx711_read_blocking(int32_t *out, uint32_t timeout_us);
static void    loadcell_push_sample(int32_t raw);
static int32_t loadcell_current_avg(void);
static float   loadcell_counts_to_grams(int32_t counts);

/* Servo bus (ONLY called from status_task — Option B) */
static esp_err_t servo_ping(uint8_t id);
static esp_err_t servo_read(uint8_t id, uint8_t addr, uint8_t n, uint8_t *out);
static esp_err_t servo_write(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t n);
static esp_err_t servo_reboot_cmd(uint8_t id);
static esp_err_t servo_set_torque(uint8_t id, bool enable);
static esp_err_t servo_set_goal(uint8_t id, uint16_t pos);
static esp_err_t servo_configure_protection(uint8_t id);

/* Fault handling */
static void fault_raise(fault_name_t f, const char *src, int32_t val, uint32_t seq);
static bool fault_is_cmd_allowed(const char *cmd);
static void fault_clear_if_resolved(bool *out_success, const char **out_reason);

/* Mode */
static void       set_mode(tray_mode_t new_mode, uint32_t seq);
static const char *mode_name(tray_mode_t m);

/* Event emitters */
static void emit_ack(uint32_t seq);
static void emit_nak(uint32_t seq, const char *reason);
static void emit_boot_event(void);
static void emit_safe_state(void);
static void emit_state_change(tray_mode_t new_mode, uint32_t seq);
static void emit_cmd_complete(uint32_t seq, const char *cmd,
                              const char *result, const char *fault_or_null,
                              uint32_t duration_ms, const char *extra_fields_json);
static void emit_heartbeat(void);
static void emit_limit_hit(const char *which, uint32_t seq);
static void emit_warning(const char *type, int32_t value, int32_t limit);
static void emit_current_peak(int32_t peak_ma, int32_t limit_ma);
static void emit_servo_status_event(uint8_t raw);
static void emit_reset_result(bool success, const char *reason, uint32_t seq);

/* Dispatch */
static void dispatch_line(const char *line, size_t len);

/* Tasks */
static void safety_task(void *arg);
static void status_task(void *arg);
static void json_cmd_task(void *arg);
static void weight_task(void *arg);
static void console_task(void *arg);

/*  ============================================================================
 *   SECTION 9  ·  UTILITY HELPERS
 *  ========================================================================= */

static uint32_t ms_now(void) {
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

static void vTaskDelay_ms(uint32_t ms) {
    /* Never round down to 0 ticks — a 0-tick delay only yields and lets a
     * high-priority loop (safety_task) starve lower tasks on its core.   */
    TickType_t t = pdMS_TO_TICKS(ms);
    vTaskDelay((t > 0) ? t : 1);
}

static const char *mode_name_str(void) {
    return (g_mode < MODE_COUNT_) ? MODE_NAME[g_mode] : "?";
}

/*  ============================================================================
 *   SECTION 10  ·  NVS PERSISTENCE
 *  ========================================================================= */

#define NVS_NS                    "tray"
#define NVS_KEY_OFFSET            "lc_offset"
#define NVS_KEY_SCALE             "lc_scale"
#define NVS_KEY_BOOTS             "boots"

static esp_err_t nvs_restore_calibration(void) {
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &g_nvs_h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }
    int32_t v;
    if (nvs_get_i32(g_nvs_h, NVS_KEY_OFFSET, &v) == ESP_OK) {
        g_lc_offset = v;
        ESP_LOGI(TAG, "restored tare offset = %" PRId32, v);
    } else {
        g_lc_offset = LC_DEFAULT_OFFSET;
    }
    if (nvs_get_i32(g_nvs_h, NVS_KEY_SCALE, &v) == ESP_OK && v > 0) {
        g_lc_scale_cpg = v;
        ESP_LOGI(TAG, "restored scale_cpg = %" PRId32, v);
    } else {
        g_lc_scale_cpg = LC_DEFAULT_SCALE_CPG;
    }
    uint32_t b;
    g_boot_count = (nvs_get_u32(g_nvs_h, NVS_KEY_BOOTS, &b) == ESP_OK) ? b : 0;
    return ESP_OK;
}

static esp_err_t nvs_save_tare(int32_t offset) {
    esp_err_t err = nvs_set_i32(g_nvs_h, NVS_KEY_OFFSET, offset);
    if (err == ESP_OK) err = nvs_commit(g_nvs_h);
    if (err != ESP_OK) ESP_LOGE(TAG, "nvs_save_tare failed: %s", esp_err_to_name(err));
    return err;
}

static esp_err_t nvs_save_scale(int32_t scale_cpg) {
    esp_err_t err = nvs_set_i32(g_nvs_h, NVS_KEY_SCALE, scale_cpg);
    if (err == ESP_OK) err = nvs_commit(g_nvs_h);
    if (err != ESP_OK) ESP_LOGE(TAG, "nvs_save_scale failed: %s", esp_err_to_name(err));
    return err;
}

static void nvs_update_boot_count(void) {
    g_boot_count++;
    esp_err_t err = nvs_set_u32(g_nvs_h, NVS_KEY_BOOTS, g_boot_count);
    if (err == ESP_OK) err = nvs_commit(g_nvs_h);
    if (err != ESP_OK) ESP_LOGW(TAG, "boot count persist failed: %s", esp_err_to_name(err));
}

/*  ============================================================================
 *   SECTION 11  ·  GPIO + LOCK
 *  ========================================================================= */

static void gpio_setup(void) {
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << PIN_RS485_DE) | (1ULL << PIN_LOCK) |
                        (1ULL << PIN_STATUS_LED) | (1ULL << PIN_FAULT_LED) |
                        (1ULL << PIN_HX711_SCK),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));

    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << PIN_LIMIT_OPEN) | (1ULL << PIN_LIMIT_CLOSED) |
                        (1ULL << PIN_HX711_DOUT),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&in_cfg));

    gpio_set_level(PIN_LOCK,       LOCK_ENGAGED_LEVEL);
    gpio_set_level(PIN_RS485_DE,   0);
    gpio_set_level(PIN_STATUS_LED, 0);
    gpio_set_level(PIN_FAULT_LED,  0);
    gpio_set_level(PIN_HX711_SCK,  0);
}

static void lock_engage(void) {
    gpio_set_level(PIN_LOCK, LOCK_ENGAGED_LEVEL);
    vTaskDelay_ms(LOCK_SETTLE_MS);
}

static void lock_disengage(void) {
    gpio_set_level(PIN_LOCK, LOCK_DISENGAGED_LEVEL);
    vTaskDelay_ms(LOCK_SETTLE_MS);
}

/*  ============================================================================
 *   SECTION 12  ·  HX711 BIT-BANG DRIVER
 *  ========================================================================= */

static bool hx711_read_blocking(int32_t *out, uint32_t timeout_us) {
    uint32_t waited_us = 0;
    while (gpio_get_level(PIN_HX711_DOUT) != 0) {
        if (waited_us >= timeout_us) return false;
        esp_rom_delay_us(50);
        waited_us += 50;
    }
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);

    uint32_t raw = 0;
    for (int i = 0; i < 24; i++) {
        gpio_set_level(PIN_HX711_SCK, 1);
        esp_rom_delay_us(1);
        raw <<= 1;
        if (gpio_get_level(PIN_HX711_DOUT)) raw |= 1;
        gpio_set_level(PIN_HX711_SCK, 0);
        esp_rom_delay_us(1);
    }
    gpio_set_level(PIN_HX711_SCK, 1);
    esp_rom_delay_us(1);
    gpio_set_level(PIN_HX711_SCK, 0);

    portEXIT_CRITICAL(&mux);

    if (raw & 0x800000UL) raw |= 0xFF000000UL;
    *out = (int32_t)raw;
    return true;
}

/*  ============================================================================
 *   SECTION 13  ·  LOAD CELL LOGIC
 *  ========================================================================= */

static void loadcell_push_sample(int32_t raw) {
    g_lc_ring[g_lc_ring_idx] = raw;
    g_lc_ring_idx = (uint8_t)((g_lc_ring_idx + 1u) & (HX711_RING_SIZE - 1u));
    if (g_lc_ring_filled < HX711_RING_SIZE) g_lc_ring_filled++;
    g_lc_last_raw = raw;
}

static int32_t loadcell_current_avg(void) {
    if (g_lc_ring_filled == 0) return 0;
    int64_t sum = 0;
    for (uint8_t i = 0; i < g_lc_ring_filled; i++) sum += g_lc_ring[i];
    return (int32_t)(sum / g_lc_ring_filled);
}

static float loadcell_counts_to_grams(int32_t counts) {
    if (g_lc_scale_cpg <= 0) return 0.0f;
    return (float)(counts - g_lc_offset) / (float)g_lc_scale_cpg;
}

/*  ============================================================================
 *   SECTION 14  ·  SERVO RS-485 DRIVER
 *     — called ONLY from status_task — Option B: single owner of the bus.
 *  ========================================================================= */

static uint8_t servo_checksum(const uint8_t *pkt, size_t n) {
    uint32_t sum = 0;
    for (size_t i = 0; i < n; i++) sum += pkt[i];
    return (uint8_t)(~sum & 0xFFu);
}

static esp_err_t servo_tx_rx(const uint8_t *tx, size_t tx_len,
                             uint8_t *rx, size_t rx_len, uint32_t rx_timeout_ms) {
    uart_flush_input(UART_SERVO);
    gpio_set_level(PIN_RS485_DE, 1);
    int w = uart_write_bytes(UART_SERVO, (const char *)tx, tx_len);
    uart_wait_tx_done(UART_SERVO, pdMS_TO_TICKS(50));
    gpio_set_level(PIN_RS485_DE, 0);
    if (w != (int)tx_len) return ESP_FAIL;
    if (rx_len == 0) return ESP_OK;
    int r = uart_read_bytes(UART_SERVO, rx, rx_len, pdMS_TO_TICKS(rx_timeout_ms));
    return (r == (int)rx_len) ? ESP_OK : ESP_ERR_TIMEOUT;
}

static esp_err_t servo_ping(uint8_t id) {
    uint8_t tx[6] = { 0xFF, 0xFF, id, 0x02, SINST_PING, 0 };
    tx[5] = servo_checksum(&tx[2], 3);
    uint8_t rx[6];
    esp_err_t err = servo_tx_rx(tx, sizeof(tx), rx, sizeof(rx), 50);
    if (err != ESP_OK) return err;
    if (rx[0] != 0xFF || rx[1] != 0xFF || rx[2] != id) return ESP_ERR_INVALID_RESPONSE;
    return ESP_OK;
}

static esp_err_t servo_read(uint8_t id, uint8_t addr, uint8_t n, uint8_t *out) {
    uint8_t tx[8] = { 0xFF, 0xFF, id, 0x04, SINST_READ, addr, n, 0 };
    tx[7] = servo_checksum(&tx[2], 5);
    uint8_t rx[32];
    if (n > sizeof(rx) - 6) return ESP_ERR_INVALID_SIZE;
    esp_err_t err = servo_tx_rx(tx, sizeof(tx), rx, 6 + n, 50);
    if (err != ESP_OK) return err;
    if (rx[0] != 0xFF || rx[1] != 0xFF || rx[2] != id) return ESP_ERR_INVALID_RESPONSE;
    if (rx[3] != n + 2) return ESP_ERR_INVALID_RESPONSE;
    if (rx[4] != 0x00) return ESP_ERR_INVALID_STATE;
    uint8_t cs = servo_checksum(&rx[2], 3 + n);
    if (cs != rx[5 + n]) return ESP_ERR_INVALID_CRC;
    memcpy(out, &rx[5], n);
    return ESP_OK;
}

static esp_err_t servo_write(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t n) {
    uint8_t tx[32];
    if ((size_t)n + 7 > sizeof(tx)) return ESP_ERR_INVALID_SIZE;
    tx[0] = 0xFF; tx[1] = 0xFF; tx[2] = id;
    tx[3] = (uint8_t)(n + 3);
    tx[4] = SINST_WRITE;
    tx[5] = addr;
    memcpy(&tx[6], data, n);
    tx[6 + n] = servo_checksum(&tx[2], 4 + n);
    uint8_t rx[6];
    esp_err_t err = servo_tx_rx(tx, 7 + n, rx, sizeof(rx), 50);
    if (err != ESP_OK) return err;
    if (rx[0] != 0xFF || rx[1] != 0xFF || rx[2] != id) return ESP_ERR_INVALID_RESPONSE;
    if (rx[4] != 0x00) return ESP_ERR_INVALID_STATE;
    return ESP_OK;
}

static esp_err_t servo_reboot_cmd(uint8_t id) {
    uint8_t tx[6] = { 0xFF, 0xFF, id, 0x02, SINST_REBOOT, 0 };
    tx[5] = servo_checksum(&tx[2], 3);
    return servo_tx_rx(tx, sizeof(tx), NULL, 0, 0);
}

static esp_err_t servo_set_torque(uint8_t id, bool enable) {
    uint8_t v = enable ? 1u : 0u;
    return servo_write(id, SREG_TORQUE_ENABLE, &v, 1);
}

static esp_err_t servo_set_goal(uint8_t id, uint16_t pos) {
    if (pos > 4095) pos = 4095;
    /* One contiguous write 0x29..0x2F: ACCEL, GOAL_POS, PWM_OPEN_SPEED, SPEED.
     * SPEED (0x2E) defaults to 0 at power-up on RSBL45-24 — goal alone won't move. */
    uint8_t d[7] = {
        (uint8_t)MOTION_ACCEL,
        (uint8_t)(pos & 0xFFu),                         (uint8_t)((pos >> 8) & 0xFFu),
        (uint8_t)(MOTION_PWM_OPEN_SPEED & 0xFFu),       (uint8_t)((MOTION_PWM_OPEN_SPEED >> 8) & 0xFFu),
        (uint8_t)(MOTION_SPEED & 0xFFu),                (uint8_t)((MOTION_SPEED >> 8) & 0xFFu),
    };
    return servo_write(id, SREG_ACCEL, d, sizeof(d));
}

/*  ============================================================================
 *   SECTION 15  ·  servo_configure_protection
 *  ========================================================================= */

static esp_err_t servo_configure_protection(uint8_t id) {
    esp_err_t err;
    uint8_t v;

    /* Response level 0 means WRITEs are applied silently with no status
     * packet — every servo_write() would then time out and be treated as
     * a failure (goal_write → BUS_BUSY fault → torque forced off).
     * Force level 1. The two writes below get no reply, so ignore errors. */
    uint8_t lvl = 0xFF;
    if (servo_read(id, SREG_RESP_LEVEL, 1, &lvl) == ESP_OK) {
        ESP_LOGI(TAG, "servo response level = %u", (unsigned)lvl);
        if (lvl == 0) {
            v = 0; (void)servo_write(id, SREG_LOCK_FLAG, &v, 1);
            v = 1; (void)servo_write(id, SREG_RESP_LEVEL, &v, 1);
            if (servo_read(id, SREG_RESP_LEVEL, 1, &lvl) == ESP_OK)
                ESP_LOGW(TAG, "servo response level forced 0 -> %u", (unsigned)lvl);
        }
    } else {
        ESP_LOGW(TAG, "servo response level read failed");
    }

    v = 0;
    err = servo_write(id, SREG_LOCK_FLAG, &v, 1);
    if (err != ESP_OK) { ESP_LOGE(TAG, "EEPROM unlock failed"); return err; }

    /* Force position (servo) mode — the servo may have been left in wheel
     * mode (1) by other firmware, in which case GOAL_POS is ignored.     */
    v = 0;
    if (servo_write(id, SREG_MODE, &v, 1) != ESP_OK)
        ESP_LOGW(TAG, "MODE write failed");

    uint16_t ic_counts = MA_TO_COUNTS(MAX_CURRENT_MA);
    uint8_t ic[2] = { (uint8_t)(ic_counts & 0xFFu), (uint8_t)((ic_counts >> 8) & 0xFFu) };
    if (servo_write(id, SREG_PROTECT_CURRENT, ic, 2) != ESP_OK)
        ESP_LOGW(TAG, "PROTECT_CURRENT write failed");

    v = (uint8_t)MAX_TEMP_C;
    if (servo_write(id, SREG_MAX_TEMP_LIMIT, &v, 1) != ESP_OK)
        ESP_LOGW(TAG, "MAX_TEMP_LIMIT write failed");

    v = 10;
    if (servo_write(id, SREG_OVERCURRENT_TIME, &v, 1) != ESP_OK)
        ESP_LOGW(TAG, "OVERCURRENT_TIME write failed");

    v = (uint8_t)(UC_VOLTAGE | UC_TEMPERATURE | UC_CURRENT | UC_OVERLOAD);
    if (servo_write(id, SREG_UNLOADING_COND, &v, 1) != ESP_OK)
        ESP_LOGW(TAG, "UNLOADING_COND write failed");

    v = 1;
    if (servo_write(id, SREG_LOCK_FLAG, &v, 1) != ESP_OK)
        ESP_LOGW(TAG, "EEPROM re-lock failed");

    ESP_LOGI(TAG, "servo_configure_protection: I≤%d mA (%u cnt)  T≤%d °C",
             MAX_CURRENT_MA, (unsigned)ic_counts, MAX_TEMP_C);
    return ESP_OK;
}

/*  ============================================================================
 *   SECTION 16  ·  FAULT HANDLING
 *  ========================================================================= */

static const char *const FAULT_ALLOWED_CMDS[] = {
    "nop", "status", "version", "diag", "weigh",
    "motor_diag", "servo_status", "raw_counts", "get_scale", "pos_query",
    "lock", "unlock", "halt", "estop", "reset_fault",
    NULL
};

static bool fault_is_cmd_allowed(const char *cmd) {
    if (!cmd) return false;
    for (int i = 0; FAULT_ALLOWED_CMDS[i]; i++) {
        if (strcmp(cmd, FAULT_ALLOWED_CMDS[i]) == 0) return true;
    }
    return false;
}

static void fault_raise(fault_name_t f, const char *src, int32_t val, uint32_t seq) {
    if (g_fault_latched) return;    /* first fault wins */
    g_fault = f;
    strncpy(g_fault_src, src ? src : "", sizeof(g_fault_src) - 1);
    g_fault_src[sizeof(g_fault_src) - 1] = '\0';
    g_fault_val = val;
    g_fault_seq = seq;
    g_fault_latched = true;
    g_mode = MODE_FAULT;
    g_homed = false;
    gpio_set_level(PIN_FAULT_LED, 1);
    ESP_LOGE(TAG, "FAULT %s src=%s val=%" PRId32, FAULT_NAME[f], g_fault_src, val);
    emit_safe_state();
    /* servo torque-off is re-asserted every 100 ms by status_task */
}

static void fault_clear_if_resolved(bool *out_success, const char **out_reason) {
    *out_success = false;
    *out_reason  = NULL;
    switch (g_fault) {
        case FAULT_OVERCURRENT:
            if (abs(g_motor_current_ma) <= WARN_CURRENT_HYST_MA) *out_success = true;
            else *out_reason = "current still above hysteresis";
            break;
        case FAULT_OVERVOLTAGE:
            if (g_motor_voltage_dv <= (MAX_VOLTAGE_DV - 5)) *out_success = true;
            else *out_reason = "voltage still above limit";
            break;
        case FAULT_UNDERVOLTAGE:
            if (g_motor_voltage_dv >= (MIN_VOLTAGE_DV + 5)) *out_success = true;
            else *out_reason = "voltage still below limit";
            break;
        case FAULT_OVERTEMP:
            if (g_motor_temp_c <= WARN_TEMP_HYST_C) *out_success = true;
            else *out_reason = "temp still above hysteresis";
            break;
        case FAULT_OVERWEIGHT:
            if (g_weight_g <= (MAX_WEIGHT_G - 10.0f)) *out_success = true;
            else *out_reason = "weight still above capacity";
            break;
        case FAULT_COMMS_LOST:
            *out_success = true;
            break;
        case FAULT_ESTOP:
        case FAULT_MOTOR_SENSOR_INVALID:
        case FAULT_MOTION_STALL:
        case FAULT_HOMING_FAILED:
        case FAULT_ENDSTOP_TIMEOUT:
        case FAULT_SENSOR_NAN:
        case FAULT_BUS_BUSY:
        case FAULT_NVS_FAILED:
        case FAULT_BROWNOUT:
        case FAULT_WDT_RESET:
            *out_success = true;    /* host asserts condition handled */
            break;
        case FAULT_NONE:
        case FAULT_COUNT_:
        default:
            *out_success = true;
            break;
    }

    if (*out_success) {
        g_fault_latched = false;
        g_fault         = FAULT_NONE;
        g_fault_src[0]  = '\0';
        g_fault_val     = 0;
        g_mode          = MODE_IDLE;
        g_trip_overcurrent  = 0;
        g_trip_overvoltage  = 0;
        g_trip_undervoltage = 0;
        g_trip_overtemp     = 0;
        g_motor_bad_reads_consec = 0;
        gpio_set_level(PIN_FAULT_LED, 0);
        ESP_LOGW(TAG, "fault cleared → IDLE (host must issue home)");
    }
}

/*  ============================================================================
 *   SECTION 17  ·  MODE TRANSITIONS
 *  ========================================================================= */

static const char *mode_name(tray_mode_t m) {
    return (m < MODE_COUNT_) ? MODE_NAME[m] : "?";
}

static void set_mode(tray_mode_t new_mode, uint32_t seq) {
    if (new_mode == g_mode) return;
    ESP_LOGI(TAG, "mode  %s → %s", mode_name(g_mode), mode_name(new_mode));
    g_mode = new_mode;
    emit_state_change(new_mode, seq);
}

/*  ============================================================================
 *   SECTION 18  ·  EVENT EMITTERS   (grabber pattern — vsnprintf into stack)
 *
 *   Every emit function is a single uart1_send_json(fmt, ...) call. No tree
 *   building. Standard trailer ,"mode":"...","t":... appears at the end of
 *   every frame via the TRAILER_FMT/TRAILER_ARGS macros.
 *   Booleans are printed as literal "true"/"false". Floats use %.2f.
 *  ========================================================================= */

#define FBOOL(b)         ((b) ? "true" : "false")

static void emit_ack(uint32_t seq) {
    uart1_send_json("{\"resp\":\"ack\",\"seq\":%u" TRAILER_FMT "}",
                    (unsigned)seq, TRAILER_ARGS);
}

static void emit_nak(uint32_t seq, const char *reason) {
    uart1_send_json("{\"resp\":\"nak\",\"seq\":%u,\"reason\":\"%s\"" TRAILER_FMT "}",
                    (unsigned)seq, reason ? reason : "unknown", TRAILER_ARGS);
}

static void emit_boot_event(void) {
    uart1_send_json("{\"event\":\"boot\",\"seq\":0,"
                    "\"fw\":\"%s\",\"git\":\"%s\",\"build\":\"%s\","
                    "\"boots\":%u,\"reset_cause\":%d" TRAILER_FMT "}",
                    FW_VERSION_STR, FW_GIT_HASH, FW_BUILD_DATE,
                    (unsigned)g_boot_count, (int)g_reset_cause, TRAILER_ARGS);
}

static void emit_safe_state(void) {
    uart1_send_json("{\"event\":\"safe_state\","
                    "\"fault\":\"%s\",\"src\":\"%s\","
                    "\"val\":%ld,\"seq\":%u" TRAILER_FMT "}",
                    FAULT_NAME[g_fault], g_fault_src,
                    (long)g_fault_val, (unsigned)g_fault_seq, TRAILER_ARGS);
}

static void emit_state_change(tray_mode_t new_mode, uint32_t seq) {
    uart1_send_json("{\"event\":\"state_change\","
                    "\"mode_to\":\"%s\",\"seq\":%u" TRAILER_FMT "}",
                    mode_name(new_mode), (unsigned)seq, TRAILER_ARGS);
}

/* emit_cmd_complete:
 *   fault_or_null  — the fault name string if result == "fault", else NULL
 *   extra_fields_json — pre-formatted JSON fragment, each caller composes with
 *                       snprintf into a local buffer. Must begin with ',' and
 *                       contain no outer braces. Pass "" or NULL for none.
 */
static void emit_cmd_complete(uint32_t seq, const char *cmd,
                              const char *result, const char *fault_or_null,
                              uint32_t duration_ms, const char *extra_fields_json)
{
    const char *extras = (extra_fields_json && extra_fields_json[0]) ? extra_fields_json : "";
    if (fault_or_null) {
        uart1_send_json("{\"event\":\"cmd_complete\",\"cmd\":\"%s\",\"seq\":%u,"
                        "\"result\":\"%s\",\"fault\":\"%s\",\"duration_ms\":%u%s"
                        TRAILER_FMT "}",
                        cmd ? cmd : "?", (unsigned)seq,
                        result ? result : "fault", fault_or_null,
                        (unsigned)duration_ms, extras, TRAILER_ARGS);
    } else {
        uart1_send_json("{\"event\":\"cmd_complete\",\"cmd\":\"%s\",\"seq\":%u,"
                        "\"result\":\"%s\",\"duration_ms\":%u%s"
                        TRAILER_FMT "}",
                        cmd ? cmd : "?", (unsigned)seq,
                        result ? result : "ok",
                        (unsigned)duration_ms, extras, TRAILER_ARGS);
    }
}

static void emit_heartbeat(void) {
    uart1_send_json("{\"event\":\"heartbeat\",\"seq\":0,"
                    "\"weight_g\":%.2f,\"weight_ok\":%s,"
                    "\"motor_ok\":%s,\"fault\":\"%s\",\"uptime_ms\":%u" TRAILER_FMT "}",
                    (double)g_weight_g, FBOOL(g_weight_ok),
                    FBOOL(g_motor_ok), FAULT_NAME[g_fault], (unsigned)ms_now(), TRAILER_ARGS);
}

static void emit_limit_hit(const char *which, uint32_t seq) {
    uart1_send_json("{\"event\":\"limit_hit\","
                    "\"which\":\"%s\",\"seq\":%u" TRAILER_FMT "}",
                    which ? which : "?", (unsigned)seq, TRAILER_ARGS);
}

static void emit_warning(const char *type, int32_t value, int32_t limit) {
    uart1_send_json("{\"event\":\"warning\",\"seq\":0,"
                    "\"type\":\"%s\",\"value\":%ld,\"limit\":%ld" TRAILER_FMT "}",
                    type ? type : "?", (long)value, (long)limit, TRAILER_ARGS);
}

static void emit_current_peak(int32_t peak_ma, int32_t limit_ma) {
    uart1_send_json("{\"event\":\"current_peak\",\"seq\":0,"
                    "\"peak_ma\":%ld,\"limit_ma\":%ld" TRAILER_FMT "}",
                    (long)peak_ma, (long)limit_ma, TRAILER_ARGS);
}

static void emit_servo_status_event(uint8_t raw) {
    uart1_send_json("{\"event\":\"servo_status\",\"seq\":0,"
                    "\"raw\":%u,\"ok\":%s,"
                    "\"voltage\":%s,\"mag_encoder\":%s,"
                    "\"overtemp\":%s,\"overcurrent\":%s,\"overload\":%s"
                    TRAILER_FMT "}",
                    (unsigned)raw, FBOOL(raw == 0),
                    FBOOL((raw & SS_VOLTAGE) != 0),
                    FBOOL((raw & SS_SENSOR) != 0),
                    FBOOL((raw & SS_TEMPERATURE) != 0),
                    FBOOL((raw & SS_CURRENT) != 0),
                    FBOOL((raw & SS_OVERLOAD) != 0),
                    TRAILER_ARGS);
}

static void emit_reset_result(bool success, const char *reason, uint32_t seq) {
    if (success || reason == NULL) {
        uart1_send_json("{\"event\":\"reset_result\","
                        "\"seq\":%u,\"success\":%s" TRAILER_FMT "}",
                        (unsigned)seq, FBOOL(success), TRAILER_ARGS);
    } else {
        uart1_send_json("{\"event\":\"reset_result\","
                        "\"seq\":%u,\"success\":false,\"reason\":\"%s\""
                        TRAILER_FMT "}",
                        (unsigned)seq, reason, TRAILER_ARGS);
    }
}

/*  ============================================================================
 *   SECTION 19  ·  MOTION PRIMITIVES
 *  ========================================================================= */

static bool post_servo_cmd(srv_cmd_type_t type, uint16_t pos, uint32_t seq) {
    srv_cmd_t c = { .type = type, .goal_pos = pos, .origin_seq = seq };
    if (!g_servo_cmd_q) return false;
    BaseType_t ok = xQueueSend(g_servo_cmd_q, &c, pdMS_TO_TICKS(100));
    if (ok != pdPASS) {
        fault_raise(FAULT_BUS_BUSY, "queue_full", 0, seq);
        return false;
    }
    return true;
}

static bool wait_for_limit(volatile bool *flag, uint32_t timeout_ms, uint32_t seq) {
    (void)seq;
    uint32_t start = ms_now();
    while ((ms_now() - start) < timeout_ms) {
        if (g_fault_latched) return false;
        if (*flag) return true;
        vTaskDelay_ms(10);
    }
    return false;
}

/*  ============================================================================
 *   SECTION 20  ·  COMMAND HANDLERS   (all 22 cmds from ICD v3.1.8 §2)
 *
 *   Signature:   void cmd_<name>(const char *raw, uint32_t seq);
 *     raw  — the full NUL-terminated JSON input line (for optional-field extraction)
 *     seq  — already extracted from "seq" by dispatch_line (0 if absent)
 *
 *   Only cmd_set_scale actually reads any payload field (scale_cpg).
 *   Every other handler ignores raw after dispatch has done its job.
 *  ========================================================================= */

/* ---------- §2.1  Diagnostics & query  (cached response) --------------- */

static void cmd_nop(const char *raw, uint32_t seq) { (void)raw; emit_ack(seq); }

static void cmd_status(const char *raw, uint32_t seq) {
    (void)raw;
    uart1_send_json("{\"resp\":\"status\",\"seq\":%u,"
                    "\"lid\":\"%s\",\"locked\":%s,\"homed\":%s,"
                    "\"weight_g\":%.2f,\"weight_ok\":%s,"
                    "\"fault\":\"%s\","
                    "\"motor\":{\"voltage_v\":%.1f,\"temp_c\":%ld,"
                              "\"current_ma\":%ld,\"ok\":%s}"
                    TRAILER_FMT "}",
                    (unsigned)seq,
                    mode_name(g_mode),
                    FBOOL(gpio_get_level(PIN_LOCK) == LOCK_ENGAGED_LEVEL),
                    FBOOL(g_homed),
                    (double)g_weight_g, FBOOL(g_weight_ok),
                    FAULT_NAME[g_fault],
                    (double)g_motor_voltage_dv / 10.0,
                    (long)g_motor_temp_c, (long)g_motor_current_ma,
                    FBOOL(g_motor_ok),
                    TRAILER_ARGS);
}

static void cmd_weigh(const char *raw, uint32_t seq) {
    (void)raw;
    uart1_send_json("{\"resp\":\"weight\",\"seq\":%u,"
                    "\"weight_g\":%.2f,\"weight_ok\":%s,"
                    "\"raw\":%ld,\"stable\":%s" TRAILER_FMT "}",
                    (unsigned)seq, (double)g_weight_g, FBOOL(g_weight_ok),
                    (long)g_lc_last_raw, FBOOL(g_weight_stable),
                    TRAILER_ARGS);
}

static void cmd_version(const char *raw, uint32_t seq) {
    (void)raw;
    uart1_send_json("{\"resp\":\"version\",\"seq\":%u,"
                    "\"version\":\"%s\",\"git\":\"%s\",\"build\":\"%s\""
                    TRAILER_FMT "}",
                    (unsigned)seq, FW_VERSION_STR, FW_GIT_HASH, FW_BUILD_DATE,
                    TRAILER_ARGS);
}

static void cmd_diag(const char *raw, uint32_t seq) {
    (void)raw;
    /* Note: emission is one uart1_send_json call. 1 KB stack buffer sized for
     * this frame specifically (the longest emission in the firmware).       */
    uart1_send_json("{\"resp\":\"diag\",\"seq\":%u,"
                    "\"uptime_ms\":%u,\"boots\":%u,\"reset_cause\":%d,"
                    "\"mode\":\"%s\",\"fault\":\"%s\",\"fault_src\":\"%s\","
                    "\"fault_latch\":%s,\"homed\":%s,"
                    "\"limits\":{\"open\":%s,\"closed\":%s},"
                    "\"comms\":{\"rx_lines\":%u,\"tx_lines\":%u,"
                              "\"parse_err\":%u,\"last_rx_ms\":%u,"
                              "\"tx_trunc\":%u},"
                    "\"motor\":{\"voltage_dv\":%ld,\"temp_c\":%ld,"
                              "\"current_ma\":%ld,\"pos\":%ld,"
                              "\"servo_status\":%u,\"last_read_ms\":%u,"
                              "\"bad_reads\":%u,"
                              "\"trip_oc\":%u,\"trip_ov\":%u,"
                              "\"trip_uv\":%u,\"trip_ot\":%u},"
                    "\"loadcell\":{\"weight_g\":%.2f,\"offset\":%ld,"
                                 "\"scale_cpg\":%ld,\"raw\":%ld,"
                                 "\"stable\":%s},"
                    "\"t\":%u}",
                    (unsigned)seq,
                    (unsigned)ms_now(), (unsigned)g_boot_count, (int)g_reset_cause,
                    mode_name(g_mode), FAULT_NAME[g_fault], g_fault_src,
                    FBOOL(g_fault_latched), FBOOL(g_homed),
                    FBOOL(g_lim_open), FBOOL(g_lim_closed),
                    (unsigned)g_rx_line_count, (unsigned)g_tx_line_count,
                    (unsigned)g_parse_err_count, (unsigned)g_last_rx_ms,
                    (unsigned)g_tx_trunc_count,
                    (long)g_motor_voltage_dv, (long)g_motor_temp_c,
                    (long)g_motor_current_ma, (long)g_motor_pos,
                    (unsigned)g_motor_servo_status, (unsigned)g_motor_last_read_ms,
                    (unsigned)g_motor_bad_reads_consec,
                    (unsigned)g_trip_overcurrent, (unsigned)g_trip_overvoltage,
                    (unsigned)g_trip_undervoltage, (unsigned)g_trip_overtemp,
                    (double)g_weight_g, (long)g_lc_offset,
                    (long)g_lc_scale_cpg, (long)g_lc_last_raw,
                    FBOOL(g_weight_stable),
                    (unsigned)ms_now());
}

static void cmd_pos_query(const char *raw, uint32_t seq) {
    (void)raw;
    uart1_send_json("{\"resp\":\"pos_query\",\"seq\":%u,"
                    "\"position\":%ld,\"success\":%s" TRAILER_FMT "}",
                    (unsigned)seq, (long)g_motor_pos,
                    FBOOL(g_motor_pos >= 0),
                    TRAILER_ARGS);
}

static void cmd_motor_diag(const char *raw, uint32_t seq) {
    (void)raw;
    uint32_t age = ms_now() - g_motor_last_read_ms;
    uart1_send_json("{\"resp\":\"motor_diag\",\"seq\":%u,"
                    "\"voltage_v\":%.1f,\"temp_c\":%ld,\"current_ma\":%ld,"
                    "\"position\":%ld,\"ok\":%s,"
                    "\"warn_current\":%s,\"warn_temp\":%s,"
                    "\"bad_reads_consec\":%u,\"last_read_age_ms\":%u"
                    TRAILER_FMT "}",
                    (unsigned)seq,
                    (double)g_motor_voltage_dv / 10.0,
                    (long)g_motor_temp_c, (long)g_motor_current_ma,
                    (long)g_motor_pos, FBOOL(g_motor_ok),
                    FBOOL(g_warn_current_latched), FBOOL(g_warn_temp_latched),
                    (unsigned)g_motor_bad_reads_consec, (unsigned)age,
                    TRAILER_ARGS);
}

static void cmd_servo_status(const char *raw, uint32_t seq) {
    (void)raw;
    uint8_t r = g_motor_servo_status;
    uart1_send_json("{\"resp\":\"servo_status\",\"seq\":%u,"
                    "\"raw\":%u,\"ok\":%s,"
                    "\"voltage\":%s,\"mag_encoder\":%s,"
                    "\"overtemp\":%s,\"overcurrent\":%s,\"overload\":%s"
                    TRAILER_FMT "}",
                    (unsigned)seq, (unsigned)r, FBOOL(r == 0),
                    FBOOL((r & SS_VOLTAGE)     != 0),
                    FBOOL((r & SS_SENSOR)      != 0),
                    FBOOL((r & SS_TEMPERATURE) != 0),
                    FBOOL((r & SS_CURRENT)     != 0),
                    FBOOL((r & SS_OVERLOAD)    != 0),
                    TRAILER_ARGS);
}

static void cmd_raw_counts(const char *raw, uint32_t seq) {
    (void)raw;
    uart1_send_json("{\"resp\":\"raw_counts\",\"seq\":%u,"
                    "\"raw\":%ld,\"avg\":%ld,\"offset\":%ld,"
                    "\"scale_cpg\":%ld,\"weight_g\":%.2f"
                    TRAILER_FMT "}",
                    (unsigned)seq,
                    (long)g_lc_last_raw, (long)g_lc_last_avg,
                    (long)g_lc_offset, (long)g_lc_scale_cpg,
                    (double)g_weight_g,
                    TRAILER_ARGS);
}

static void cmd_get_scale(const char *raw, uint32_t seq) {
    (void)raw;
    uart1_send_json("{\"resp\":\"get_scale\",\"seq\":%u,"
                    "\"scale_cpg\":%ld" TRAILER_FMT "}",
                    (unsigned)seq, (long)g_lc_scale_cpg, TRAILER_ARGS);
}

static void cmd_selftest(const char *raw, uint32_t seq) {
    (void)raw;
    bool gpio_ok     = true;
    bool lock_ok     = (gpio_get_level(PIN_LOCK) == LOCK_ENGAGED_LEVEL);
    bool nvs_ok      = (g_nvs_h != 0);
    bool limits_ok   = true;
    bool loadcell_ok = (g_weight_ok && g_lc_ring_filled > 0);
    bool servo_ok    = g_motor_ok && g_motor_bad_reads_consec == 0;
    bool overall     = gpio_ok && lock_ok && nvs_ok && limits_ok && loadcell_ok && servo_ok;

    uart1_send_json("{\"resp\":\"selftest\",\"seq\":%u,"
                    "\"gpio\":%s,\"servo\":%s,\"loadcell\":%s,"
                    "\"lock\":%s,\"nvm\":%s,\"limits\":%s,"
                    "\"overall\":\"%s\"" TRAILER_FMT "}",
                    (unsigned)seq,
                    FBOOL(gpio_ok), FBOOL(servo_ok), FBOOL(loadcell_ok),
                    FBOOL(lock_ok), FBOOL(nvs_ok), FBOOL(limits_ok),
                    overall ? "pass" : "fail",
                    TRAILER_ARGS);
}

/* ---------- §2.2  Load-cell configuration ------------------------------ */

static void cmd_tare(const char *raw, uint32_t seq) {
    (void)raw;
    if (g_mode != MODE_CLOSED_LOCKED) { emit_nak(seq, "bad_state"); return; }
    if (!g_weight_stable)             { emit_nak(seq, "not_stable"); return; }

    int64_t sum = 0;
    uint8_t got = 0;
    for (uint8_t i = 0; i < HX711_TARE_SAMPLES * 2 && got < HX711_TARE_SAMPLES; i++) {
        int32_t r;
        xSemaphoreTake(g_lc_mutex, portMAX_DELAY);
        bool ok = hx711_read_blocking(&r, HX711_READ_TIMEOUT_US);
        xSemaphoreGive(g_lc_mutex);
        if (ok) { sum += r; got++; }
        else    { vTaskDelay_ms(20); }
    }

    if (got < HX711_TARE_SAMPLES) {
        uart1_send_json("{\"resp\":\"tare\",\"seq\":%u,"
                        "\"success\":false,\"reason\":\"sampling_failed\""
                        TRAILER_FMT "}",
                        (unsigned)seq, TRAILER_ARGS);
        return;
    }

    int32_t new_offset = (int32_t)(sum / got);
    xSemaphoreTake(g_lc_mutex, portMAX_DELAY);
    g_lc_offset = new_offset;
    xSemaphoreGive(g_lc_mutex);

    if (nvs_save_tare(new_offset) != ESP_OK) {
        fault_raise(FAULT_NVS_FAILED, "tare", 0, seq);
    }

    uart1_send_json("{\"resp\":\"tare\",\"seq\":%u,"
                    "\"success\":true,\"tare_counts\":%ld,\"samples\":%u"
                    TRAILER_FMT "}",
                    (unsigned)seq, (long)new_offset, (unsigned)got,
                    TRAILER_ARGS);
}

static void cmd_set_scale(const char *raw, uint32_t seq) {
    int32_t new_scale = 0;
    if (!parse_json_int_field(raw, "scale_cpg", &new_scale) || new_scale <= 0) {
        emit_nak(seq, "bad_param");
        return;
    }

    xSemaphoreTake(g_lc_mutex, portMAX_DELAY);
    g_lc_scale_cpg = new_scale;
    xSemaphoreGive(g_lc_mutex);

    if (nvs_save_scale(new_scale) != ESP_OK) {
        fault_raise(FAULT_NVS_FAILED, "set_scale", 0, seq);
    }

    uart1_send_json("{\"resp\":\"set_scale\",\"seq\":%u,"
                    "\"success\":true,\"scale_cpg\":%ld" TRAILER_FMT "}",
                    (unsigned)seq, (long)new_scale, TRAILER_ARGS);
}

/* ---------- §2.3  Motor & safety utilities ----------------------------- */

static void cmd_estop(const char *raw, uint32_t seq) {
    (void)raw;
    emit_ack(seq);
    fault_raise(FAULT_ESTOP, "host_cmd", 0, seq);
}

static void cmd_reset_fault(const char *raw, uint32_t seq) {
    (void)raw;
    emit_ack(seq);
    if (!g_fault_latched) { emit_reset_result(true, NULL, seq); return; }
    bool ok; const char *reason;
    fault_clear_if_resolved(&ok, &reason);
    emit_reset_result(ok, reason, seq);
}

static void cmd_halt(const char *raw, uint32_t seq) {
    (void)raw;
    emit_ack(seq);
    (void)post_servo_cmd(SRV_CMD_TORQUE_OFF, 0, seq);
    bool was_moving = (g_mode == MODE_OPENING || g_mode == MODE_CLOSING ||
                       g_mode == MODE_HOMING  || g_mode == MODE_UNLOCKING ||
                       g_mode == MODE_LOCKING);
    g_homed = false;
    set_mode(MODE_IDLE, seq);
    if (was_moving) {
        emit_cmd_complete(seq, "halt", "halted", NULL, 0, NULL);
    }
}

static void cmd_servo_reboot(const char *raw, uint32_t seq) {
    (void)raw;
    g_motor_supervision_suspended_until_ms = ms_now() + SERVO_REBOOT_WAIT_MS * 2;
    if (!post_servo_cmd(SRV_CMD_REBOOT, 0, seq)) {
        emit_nak(seq, "bus_busy");
        return;
    }
    emit_ack(seq);
}

/* ---------- §2.4  Manual overrides ------------------------------------- */

static void cmd_lock(const char *raw, uint32_t seq) {
    (void)raw;
    gpio_set_level(PIN_LOCK, LOCK_ENGAGED_LEVEL);
    emit_ack(seq);
}

static void cmd_unlock(const char *raw, uint32_t seq) {
    (void)raw;
    gpio_set_level(PIN_LOCK, LOCK_DISENGAGED_LEVEL);
    emit_ack(seq);
}

/* ---------- §2.5  Motion commands (3-phase async, inline) -------------- */

static void cmd_home(const char *raw, uint32_t seq) {
    (void)raw;
    if (!(g_mode == MODE_IDLE || g_mode == MODE_CLOSED_LOCKED)) {
        emit_nak(seq, "bad_state");
        return;
    }
    uint32_t t0 = ms_now();
    emit_ack(seq);

    lock_disengage();
    set_mode(MODE_HOMING, seq);
    if (!post_servo_cmd(SRV_CMD_TORQUE_ON, 0, seq)) return;
    if (!post_servo_cmd(SRV_CMD_GOAL, SERVO_POS_CLOSED, seq)) return;

    if (!wait_for_limit(&g_lim_closed, ENDSTOP_TIMEOUT_MS, seq)) {
        if (!g_fault_latched) {
            fault_raise(FAULT_HOMING_FAILED, "endstop_timeout", 0, seq);
        }
        emit_cmd_complete(seq, "home", "fault", FAULT_NAME[g_fault], ms_now() - t0, NULL);
        return;
    }

    set_mode(MODE_LOCKING, seq);
    lock_engage();
    (void)post_servo_cmd(SRV_CMD_TORQUE_OFF, 0, seq);
    set_mode(MODE_CLOSED_LOCKED, seq);
    g_homed = true;

    emit_cmd_complete(seq, "home", "ok", NULL, ms_now() - t0,
                      ",\"homed\":true");
}

static void cmd_open(const char *raw, uint32_t seq) {
    (void)raw;
    if (g_mode != MODE_CLOSED_LOCKED) { emit_nak(seq, "bad_state"); return; }
    if (!g_homed)                     { emit_nak(seq, "not_homed"); return; }

    /* Capacity check — rolling 20 ms average, NO stability requirement */
    if (g_weight_g > MAX_WEIGHT_G) {
        char extras[96];
        int n = snprintf(extras, sizeof(extras),
                         ",\"weight_g\":%.2f,\"max_weight_g\":%.1f",
                         (double)g_weight_g, (double)MAX_WEIGHT_G);
        if (n < 0 || n >= (int)sizeof(extras)) extras[0] = '\0';
        emit_nak(seq, "capacity_reached");
        emit_cmd_complete(seq, "open", "fault", "capacity_reached", 0, extras);
        return;
    }

    uint32_t t0 = ms_now();
    emit_ack(seq);

    set_mode(MODE_UNLOCKING, seq);
    lock_disengage();
    set_mode(MODE_OPENING, seq);
    if (!post_servo_cmd(SRV_CMD_TORQUE_ON, 0, seq)) return;
    if (!post_servo_cmd(SRV_CMD_GOAL, SERVO_POS_OPEN, seq)) return;

    if (!wait_for_limit(&g_lim_open, ENDSTOP_TIMEOUT_MS, seq)) {
        if (!g_fault_latched) {
            fault_raise(FAULT_ENDSTOP_TIMEOUT, "open_timeout", 0, seq);
        }
        emit_cmd_complete(seq, "open", "fault",
                          (g_fault == FAULT_ENDSTOP_TIMEOUT) ? "open_timeout" : FAULT_NAME[g_fault],
                          ms_now() - t0, NULL);
        return;
    }

    set_mode(MODE_OPEN, seq);
    emit_cmd_complete(seq, "open", "ok", NULL, ms_now() - t0,
                      ",\"tray\":\"opened\"");
}

static void cmd_close(const char *raw, uint32_t seq) {
    (void)raw;
    if (g_mode != MODE_OPEN) { emit_nak(seq, "bad_state"); return; }

    uint32_t t0 = ms_now();
    emit_ack(seq);

    set_mode(MODE_CLOSING, seq);
    if (!post_servo_cmd(SRV_CMD_TORQUE_ON, 0, seq)) return;
    if (!post_servo_cmd(SRV_CMD_GOAL, SERVO_POS_CLOSED, seq)) return;

    if (!wait_for_limit(&g_lim_closed, ENDSTOP_TIMEOUT_MS, seq)) {
        if (!g_fault_latched) {
            fault_raise(FAULT_ENDSTOP_TIMEOUT, "close_timeout", 0, seq);
        }
        emit_cmd_complete(seq, "close", "fault",
                          (g_fault == FAULT_ENDSTOP_TIMEOUT) ? "close_timeout" : FAULT_NAME[g_fault],
                          ms_now() - t0, NULL);
        return;
    }

    set_mode(MODE_LOCKING, seq);
    lock_engage();
    (void)post_servo_cmd(SRV_CMD_TORQUE_OFF, 0, seq);

    /* INSTANT weight read — close skips MODE_WEIGHING per ICD v3.1.8 */
    float weight = g_weight_g;

    set_mode(MODE_CLOSED_LOCKED, seq);

    char extras[80];
    int n = snprintf(extras, sizeof(extras),
                     ",\"tray\":\"closed\",\"weight_g\":%.2f", (double)weight);
    if (n < 0 || n >= (int)sizeof(extras)) extras[0] = '\0';
    emit_cmd_complete(seq, "close", "ok", NULL, ms_now() - t0, extras);
}

/*  ============================================================================
 *   SECTION 21  ·  COMMAND DISPATCHER
 *  ========================================================================= */

typedef void (*cmd_handler_t)(const char *raw, uint32_t seq);

typedef struct {
    const char    *name;
    cmd_handler_t  handler;
} cmd_entry_t;

static const cmd_entry_t CMD_TABLE[] = {
    { "nop",          cmd_nop          },
    { "status",       cmd_status       },
    { "weigh",        cmd_weigh        },
    { "version",      cmd_version      },
    { "diag",         cmd_diag         },
    { "pos_query",    cmd_pos_query    },
    { "motor_diag",   cmd_motor_diag   },
    { "servo_status", cmd_servo_status },
    { "raw_counts",   cmd_raw_counts   },
    { "get_scale",    cmd_get_scale    },
    { "selftest",     cmd_selftest     },
    { "tare",         cmd_tare         },
    { "set_scale",    cmd_set_scale    },
    { "estop",        cmd_estop        },
    { "reset_fault",  cmd_reset_fault  },
    { "halt",         cmd_halt         },
    { "servo_reboot", cmd_servo_reboot },
    { "lock",         cmd_lock         },
    { "unlock",       cmd_unlock       },
    { "home",         cmd_home         },
    { "open",         cmd_open         },
    { "close",        cmd_close        },
};
#define CMD_TABLE_LEN  ((sizeof(CMD_TABLE) / sizeof(CMD_TABLE[0])))

static void dispatch_line(const char *line, size_t len) {
    (void)len;
    char cmd[32];
    if (!parse_json_cmd(line, cmd, sizeof(cmd))) {
        g_parse_err_count++;
        return;
    }

    int32_t seq_i = 0;
    uint32_t seq = 0;
    if (parse_json_int_field(line, "seq", &seq_i) && seq_i >= 0) {
        seq = (uint32_t)seq_i;
    }

    g_last_rx_ms = ms_now();
    g_rx_line_count++;

    if (g_fault_latched && !fault_is_cmd_allowed(cmd)) {
        emit_nak(seq, "fault_latched");
        return;
    }

    for (size_t i = 0; i < CMD_TABLE_LEN; i++) {
        if (strcmp(cmd, CMD_TABLE[i].name) == 0) {
            CMD_TABLE[i].handler(line, seq);
            return;
        }
    }

    emit_nak(seq, "unknown_cmd");
}

/*  ============================================================================
 *   SECTION 22  ·  safety_task      P4 · core 0 · 1 ms
 *     - WDT reset
 *     - limit_poll (8-sample debounce) + limit_hit edge emission
 *     - comms watchdog (3 s host silence → COMMS_LOST)
 *     NO ESTOP pin check — e-stop is command-only (ICD v3.1.8).
 *  ========================================================================= */

static void safety_task(void *arg) {
    (void)arg;
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    uint32_t boot_start = ms_now();
    g_last_rx_ms = boot_start;

    for (;;) {
        esp_task_wdt_reset();

        int lo = (gpio_get_level(PIN_LIMIT_OPEN)   == 0);
        int lc = (gpio_get_level(PIN_LIMIT_CLOSED) == 0);

        if (lo) { if (g_lim_open_dbnc   < LIMIT_DEBOUNCE_SAMPLES) g_lim_open_dbnc++;   }
        else    { if (g_lim_open_dbnc   > 0)                      g_lim_open_dbnc--;   }
        if (lc) { if (g_lim_closed_dbnc < LIMIT_DEBOUNCE_SAMPLES) g_lim_closed_dbnc++; }
        else    { if (g_lim_closed_dbnc > 0)                      g_lim_closed_dbnc--; }

        g_lim_open   = (g_lim_open_dbnc   >= LIMIT_DEBOUNCE_SAMPLES);
        g_lim_closed = (g_lim_closed_dbnc >= LIMIT_DEBOUNCE_SAMPLES);

        bool past_grace = (ms_now() - boot_start) > BOOT_GRACE_MS;
        if (past_grace) {
            if (g_lim_open   && !g_lim_open_prev)   emit_limit_hit("open",   0);
            if (g_lim_closed && !g_lim_closed_prev) emit_limit_hit("closed", 0);
        }
        g_lim_open_prev   = g_lim_open;
        g_lim_closed_prev = g_lim_closed;

        /* ---------------------------------------------------------------- *
         * Comms watchdog REMOVED by design decision (v1.3.2).              *
         *   Host silence no longer triggers FAULT_COMMS_LOST.              *
         *   g_last_rx_ms is still tracked and exposed via diag for         *
         *   host-side debugging ("time since last valid cmd").             *
         *   SOP §14 deviation — see file header comment.                   *
         * ---------------------------------------------------------------- */
        (void)past_grace;    /* now only used by limit-switch boot grace */

        vTaskDelay_ms(SAFETY_PERIOD_MS);
    }
}

/*  ============================================================================
 *   SECTION 23  ·  status_task     P3 · core 0 · 100 ms
 *     SOLE OWNER OF UART2 (servo bus — Option B).
 *     1. Re-assert servo_torque(false) every tick while fault_latched
 *     2. Motor telemetry (0x45 / 0x3E / 0x3F) + threshold checks
 *     3. 80 % warning band + peak current tracking
 *     4. 1 Hz slow poll of servo_status (0x41) + servo_status event
 *     5. 1 Hz heartbeat emission
 *     6. servo_cmd_q drain
 *  ========================================================================= */

static void status_eval_current(int32_t current_ma, uint32_t seq_hint) {
    if (abs(current_ma) > MAX_CURRENT_MA) {
        if (g_trip_overcurrent < 255) g_trip_overcurrent++;
        if (g_trip_overcurrent >= FAULT_TRIP_COUNT) {
            fault_raise(FAULT_OVERCURRENT, "current_over", current_ma, seq_hint);
            return;
        }
    } else {
        g_trip_overcurrent = 0;
    }

    int32_t mag = abs(current_ma);
    if (!g_warn_current_latched && mag > WARN_CURRENT_MA) {
        g_warn_current_latched = true;
        emit_warning("current", mag, MAX_CURRENT_MA);
    } else if (g_warn_current_latched && mag < WARN_CURRENT_HYST_MA) {
        g_warn_current_latched = false;
    }

    if (mag > PEAK_TRACK_START_MA) {
        g_motor_tracking_peak = true;
        if (mag > g_motor_peak_ma) g_motor_peak_ma = mag;
    } else if (g_motor_tracking_peak && mag < PEAK_TRACK_END_MA) {
        emit_current_peak(g_motor_peak_ma, MAX_CURRENT_MA);
        g_motor_peak_ma       = 0;
        g_motor_tracking_peak = false;
    }
}

static void status_eval_voltage(int32_t voltage_dv, uint32_t seq_hint) {
    if (voltage_dv > MAX_VOLTAGE_DV) {
        if (g_trip_overvoltage < 255) g_trip_overvoltage++;
        if (g_trip_overvoltage >= FAULT_TRIP_COUNT) {
            fault_raise(FAULT_OVERVOLTAGE, "voltage_over", voltage_dv, seq_hint);
            return;
        }
    } else {
        g_trip_overvoltage = 0;
    }

    if (voltage_dv < MIN_VOLTAGE_DV) {
        if (g_trip_undervoltage < 255) g_trip_undervoltage++;
        if (g_trip_undervoltage >= FAULT_TRIP_COUNT) {
            fault_raise(FAULT_UNDERVOLTAGE, "voltage_under", voltage_dv, seq_hint);
            return;
        }
    } else {
        g_trip_undervoltage = 0;
    }
}

static void status_eval_temp(int32_t temp_c, uint32_t seq_hint) {
    if (temp_c > MAX_TEMP_C) {
        if (g_trip_overtemp < 255) g_trip_overtemp++;
        if (g_trip_overtemp >= FAULT_TRIP_COUNT) {
            fault_raise(FAULT_OVERTEMP, "temp_over", temp_c, seq_hint);
            return;
        }
    } else {
        g_trip_overtemp = 0;
    }

    if (!g_warn_temp_latched && temp_c > WARN_TEMP_C) {
        g_warn_temp_latched = true;
        emit_warning("temperature", temp_c, MAX_TEMP_C);
    } else if (g_warn_temp_latched && temp_c < WARN_TEMP_HYST_C) {
        g_warn_temp_latched = false;
    }
}

static void status_execute_queue_cmd(const srv_cmd_t *c) {
    switch (c->type) {
        case SRV_CMD_GOAL: {
            esp_err_t err = servo_set_goal(SERVO_ID, c->goal_pos);
            if (err != ESP_OK) {
                /* No/garbled status reply — check whether the goal landed anyway. */
                uint8_t rb[2];
                uint16_t want = (c->goal_pos > 4095) ? 4095 : c->goal_pos;
                if (servo_read(SERVO_ID, SREG_GOAL_POS, 2, rb) == ESP_OK &&
                    ((uint16_t)rb[0] | ((uint16_t)rb[1] << 8)) == want) {
                    ESP_LOGW(TAG, "goal_write reply %s but readback ok (%u)",
                             esp_err_to_name(err), (unsigned)want);
                } else {
                    ESP_LOGE(TAG, "goal_write failed: %s", esp_err_to_name(err));
                    fault_raise(FAULT_BUS_BUSY, "goal_write", (int32_t)c->goal_pos, c->origin_seq);
                }
            }
            break;
        }
        case SRV_CMD_TORQUE_OFF:
            (void)servo_set_torque(SERVO_ID, false);
            break;
        case SRV_CMD_TORQUE_ON:
            if (g_fault_latched) break;
            if (servo_set_torque(SERVO_ID, true) != ESP_OK)
                ESP_LOGW(TAG, "torque_on write got no valid reply");
            break;
        case SRV_CMD_REBOOT:
            (void)servo_reboot_cmd(SERVO_ID);
            break;
        default:
            ESP_LOGW(TAG, "unknown servo_cmd type %d", (int)c->type);
            break;
    }
}

static void status_task(void *arg) {
    (void)arg;
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    uint32_t last_hb_ms         = 0;
    uint32_t last_servo_stat_ms = 0;
    uint8_t  last_servo_stat    = 0xFF;
    uint32_t last_bad_read_warn_ms = 0;

    /* Keep feeding the WDT while the servo is unpowered/absent. */
    while (!g_motor_bus_ready) { esp_task_wdt_reset(); vTaskDelay_ms(10); }

    for (;;) {
        esp_task_wdt_reset();

        /* 1. Re-assert torque off while fault_latched */
        if (g_fault_latched) {
            (void)servo_set_torque(SERVO_ID, false);
        }

        /* 2. Motor telemetry */
        bool past_grace = (ms_now() > BOOT_GRACE_MS);
        bool supervise  = past_grace &&
                          (ms_now() >= g_motor_supervision_suspended_until_ms);

        if (supervise) {
            uint8_t rbuf[2];
            bool good_cycle = true;

            if (servo_read(SERVO_ID, SREG_PRESENT_CURRENT, 2, rbuf) == ESP_OK) {
                uint16_t raw = (uint16_t)rbuf[0] | ((uint16_t)rbuf[1] << 8);
                if (raw == SENTINEL_WORD) {
                    good_cycle = false;
                } else {
                    uint16_t mag = raw & CURRENT_MAG_MASK;
                    if (mag >= CURRENT_SAT_COUNTS) {
                        fault_raise(FAULT_OVERCURRENT, "current_sat", (int32_t)mag, 0);
                    } else {
                        int32_t ma = ((int32_t)mag * CURRENT_LSB_UA) / 1000;
                        if (raw & CURRENT_SIGN_BIT) ma = -ma;
                        g_motor_current_ma = ma;
                        status_eval_current(ma, 0);
                    }
                }
            } else {
                good_cycle = false;
            }

            if (!g_fault_latched &&
                servo_read(SERVO_ID, SREG_PRESENT_VOLT, 1, rbuf) == ESP_OK) {
                uint8_t raw = rbuf[0];
                if (raw == SENTINEL_BYTE) {
                    good_cycle = false;
                } else if (raw >= VOLTAGE_SAT_COUNTS) {
                    fault_raise(FAULT_OVERVOLTAGE, "voltage_sat", (int32_t)raw, 0);
                } else {
                    g_motor_voltage_dv = (int32_t)raw;
                    status_eval_voltage((int32_t)raw, 0);
                }
            } else if (!g_fault_latched) {
                good_cycle = false;
            }

            if (!g_fault_latched &&
                servo_read(SERVO_ID, SREG_PRESENT_TEMP, 1, rbuf) == ESP_OK) {
                uint8_t raw = rbuf[0];
                if (raw == SENTINEL_BYTE) {
                    good_cycle = false;
                } else {
                    g_motor_temp_c = (int32_t)raw;
                    status_eval_temp((int32_t)raw, 0);
                }
            } else if (!g_fault_latched) {
                good_cycle = false;
            }

            if (!g_fault_latched &&
                servo_read(SERVO_ID, SREG_PRESENT_POS, 2, rbuf) == ESP_OK) {
                int32_t p = (int32_t)rbuf[0] | ((int32_t)rbuf[1] << 8);
                if (p >= 0 && p <= 4095) g_motor_pos = p;
            }

            if (good_cycle) {
                g_motor_bad_reads_consec = 0;
                g_motor_ok               = true;
                g_motor_last_read_ms     = ms_now();
            } else {
                if (g_motor_bad_reads_consec < 255) g_motor_bad_reads_consec++;
                if (g_motor_bad_reads_consec >= BAD_READ_LIMIT) {
                    tray_mode_t m = g_mode;
                    bool in_motion = (m == MODE_HOMING   || m == MODE_OPENING ||
                                      m == MODE_CLOSING  || m == MODE_UNLOCKING ||
                                      m == MODE_LOCKING);
                    if (in_motion) {
                        fault_raise(FAULT_MOTOR_SENSOR_INVALID, "bad_reads",
                                    (int32_t)g_motor_bad_reads_consec, 0);
                    } else if ((ms_now() - last_bad_read_warn_ms) >= BAD_READ_WARN_PERIOD_MS) {
                        /* Static state: warn only, don't latch (flaky bench RS-485). */
                        last_bad_read_warn_ms = ms_now();
                        ESP_LOGW(TAG, "servo telemetry bad_reads=%u in %s (not latched)",
                                 (unsigned)g_motor_bad_reads_consec, mode_name(m));
                    }
                }
                g_motor_ok = false;
            }

            /* 1 Hz slow poll of servo status */
            if ((ms_now() - last_servo_stat_ms) >= SERVO_STATUS_POLL_MS) {
                last_servo_stat_ms = ms_now();
                if (servo_read(SERVO_ID, SREG_SERVO_STATUS, 1, rbuf) == ESP_OK) {
                    uint8_t raw = rbuf[0];
                    g_motor_servo_status = raw;
                    if (raw != last_servo_stat) {
                        last_servo_stat = raw;
                        emit_servo_status_event(raw);
                        if (!g_fault_latched) {
                            if (raw & SS_VOLTAGE)
                                fault_raise(FAULT_OVERVOLTAGE,  "servo_flag", 0, 0);
                            else if (raw & SS_TEMPERATURE)
                                fault_raise(FAULT_OVERTEMP,     "servo_flag", 0, 0);
                            else if (raw & SS_CURRENT)
                                fault_raise(FAULT_OVERCURRENT,  "servo_flag", 0, 0);
                            else if (raw & SS_SENSOR)
                                fault_raise(FAULT_MOTOR_SENSOR_INVALID, "servo_flag", 0, 0);
                            else if (raw & SS_OVERLOAD)
                                fault_raise(FAULT_MOTION_STALL, "servo_flag", 0, 0);
                        }
                    }
                }
            }
        }

        /* 3. Heartbeat (HEARTBEAT_PERIOD_MS) */
        if ((ms_now() - last_hb_ms) >= HEARTBEAT_PERIOD_MS) {
            last_hb_ms = ms_now();
            emit_heartbeat();
            gpio_set_level(PIN_STATUS_LED, 1 - gpio_get_level(PIN_STATUS_LED));
        }

        /* 4. Drain servo_cmd_q (all pending commands this tick) */
        srv_cmd_t c;
        while (xQueueReceive(g_servo_cmd_q, &c, 0) == pdPASS) {
            if (g_fault_latched && c.type != SRV_CMD_TORQUE_OFF) {
                ESP_LOGW(TAG, "servo_cmd dropped (fault_latched) type=%d", c.type);
            } else {
                status_execute_queue_cmd(&c);
            }
        }

        vTaskDelay_ms(STATUS_PERIOD_MS);
    }
}

/*  ============================================================================
 *   SECTION 24  ·  json_cmd_task     P2 · core 1 · event-driven
 *  ========================================================================= */

static void json_cmd_task(void *arg) {
    (void)arg;
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    static char   line[JSON_LINE_MAX_IN + 1];
    static size_t pos = 0;
    uint8_t rx[128];

    for (;;) {
        esp_task_wdt_reset();

        int n = uart_read_bytes(UART_JSON, rx, sizeof(rx), pdMS_TO_TICKS(50));
        if (n < 0) { vTaskDelay_ms(10); continue; }

        for (int i = 0; i < n; i++) {
            char c = (char)rx[i];
            if (c == '\n' || c == '\r') {
                if (pos > 0) {
                    line[pos] = '\0';
                    dispatch_line(line, pos);
                    pos = 0;
                }
            } else if (pos < JSON_LINE_MAX_IN) {
                line[pos++] = c;
            } else {
                g_parse_err_count++;
                pos = 0;      /* overflow — drop line */
            }
        }
    }
}

/*  ============================================================================
 *   SECTION 24.5  ·  console_task     P1 · core 1 · event-driven  (UART0)
 *
 *   Bench convenience path. Reads short plain-text commands from the native
 *   USB console (UART0, the port idf.py monitor is attached to) and dispatches
 *   them through the exact same CMD_TABLE as JSON commands on UART1.
 *
 *   Line formats accepted:
 *     open                             → {"cmd":"open"}
 *     status                           → {"cmd":"status"}
 *     set_scale 150                    → {"cmd":"set_scale","scale_cpg":150}
 *     {"cmd":"home","seq":4}           → passed through verbatim
 *     help  |  ?                       → prints the command list
 *
 *   Backspace / DEL (0x08 / 0x7F) edits the line. Enter / newline dispatches.
 *   All responses are mirrored to UART0 by uart1_send_json, so you see the
 *   reply in the same monitor window.
 *  ========================================================================= */

/* Write to UART0 and native USB-Serial-JTAG (whichever installed OK). */
static void console_write_all(const char *s, size_t len) {
    if (g_console_ready) uart_write_bytes(UART_CONSOLE, s, len);
    if (g_usbjtag_ready) usb_serial_jtag_write_bytes(s, len, pdMS_TO_TICKS(20));
}

/* Read one chunk from UART0 with a 100 ms timeout.
 * Returns byte count (0 on timeout, >0 on data). dst is null-terminated. */
static int console_read_any(uint8_t *dst, size_t cap) {
    if (!g_console_ready) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return 0;
    }
    int n = 0;
    if (g_usbjtag_ready) {
        n = usb_serial_jtag_read_bytes(dst, cap - 1, pdMS_TO_TICKS(50));
    }
    if (n <= 0) {
        n = uart_read_bytes(UART_CONSOLE, dst, cap - 1, pdMS_TO_TICKS(50));
    }
    if (n > 0) dst[n] = '\0';
    return (n > 0) ? n : 0;
}

/* Synthesize a JSON frame from a short cmd name and dispatch through the
 * same CMD_TABLE as UART1 input. SOP §8.1: static buffer, bounded. */
static void console_dispatch_short(const char *name) {
    static char synth[64];
    int n = snprintf(synth, sizeof(synth), "{\"cmd\":\"%s\"}", name);
    if (n > 0 && n < (int)sizeof(synth)) dispatch_line(synth, (size_t)n);
}

/* set_scale variant: extract the numeric argument that followed the keyword
 * and build a JSON frame carrying it. */
static void console_dispatch_set_scale(const char *after_keyword) {
    const char *p = after_keyword;
    while (*p == ' ' || *p == '\t' || *p == '=') p++;
    bool neg = false;
    if (*p == '-') { neg = true; p++; }
    if (*p < '0' || *p > '9') {
        const char *w =
            "\r\n[console] set_scale: expected numeric value (e.g. 'set_scale 100')\r\n";
        console_write_all(w, strlen(w));
        return;
    }
    int val = 0, digits = 0;
    while (*p >= '0' && *p <= '9' && digits < 7) {
        val = val * 10 + (*p - '0');
        p++; digits++;
    }
    if (neg) val = -val;
    static char synth[80];
    int n = snprintf(synth, sizeof(synth),
                     "{\"cmd\":\"set_scale\",\"scale_cpg\":%d}", val);
    if (n > 0 && n < (int)sizeof(synth)) dispatch_line(synth, (size_t)n);
}

static void console_print_help(void) {
    static const char help[] =
        "\r\n--- sediment tray console (type keyword, press Enter) ---\r\n"
        "  Motion:   open  close  home\r\n"
        "  Query:    status  weigh  version  diag  nop\r\n"
        "            pos_query  motor_diag  servo_status  raw_counts\r\n"
        "            get_scale  selftest\r\n"
        "  Loadcell: tare   set_scale <N>\r\n"
        "  Safety:   estop  reset_fault  halt  servo_reboot\r\n"
        "  Manual:   lock   unlock\r\n"
        "  Meta:     help | ?\r\n"
        "  JSON also accepted on this port.\r\n"
        "----------------------------------------------------------\r\n";
    console_write_all(help, sizeof(help) - 1);
}

/* Case-insensitive substring match (needle in haystack). */
static const char *strcasestr_safe(const char *hay, const char *needle) {
    if (!hay || !needle || !*needle) return NULL;
    size_t nlen = strlen(needle);
    for (const char *p = hay; *p; p++) {
        size_t i;
        for (i = 0; i < nlen; i++) {
            char a = p[i]; char b = needle[i];
            if (a >= 'A' && a <= 'Z') a = (char)(a + ('a' - 'A'));
            if (b >= 'A' && b <= 'Z') b = (char)(b + ('a' - 'A'));
            if (!a || a != b) break;
        }
        if (i == nlen) return p;
    }
    return NULL;
}

/* ----------------------------------------------------------------------------
 *  console_handle_line — keyword matcher for one complete console line.
 *
 *    - search with strcasestr_safe for each keyword
 *    - longer / more specific keywords are checked FIRST to avoid matching
 *      their short-form prefix inside them (motor_diag before diag,
 *      servo_status before status, unlock before lock, etc.)
 *    - JSON pass-through: a '{' anywhere in the line routes to dispatch_line
 * -------------------------------------------------------------------------- */
static void console_handle_line(char *buf) {
    {
        /* JSON pass-through — if the line contains '{', treat the slice
         * from '{' onwards as a JSON frame and feed it to dispatch_line. */
        char *brace = strchr(buf, '{');
        if (brace != NULL) {
            dispatch_line(brace, strlen(brace));
            return;
        }

        /* Help first (short word, no collisions worth guarding) */
        if (strcasestr_safe(buf, "help") || strchr(buf, '?')) {
            console_print_help();
            return;
        }

        /* --- COMPOUND / UNDERSCORED commands FIRST (contain shorter words) --- */
        if (strcasestr_safe(buf, "motor_diag"))   { console_dispatch_short("motor_diag");   return; }
        if (strcasestr_safe(buf, "servo_status")) { console_dispatch_short("servo_status"); return; }
        if (strcasestr_safe(buf, "servo_reboot")) { console_dispatch_short("servo_reboot"); return; }
        if (strcasestr_safe(buf, "reset_fault"))  { console_dispatch_short("reset_fault");  return; }
        if (strcasestr_safe(buf, "raw_counts"))   { console_dispatch_short("raw_counts");   return; }
        if (strcasestr_safe(buf, "pos_query"))    { console_dispatch_short("pos_query");    return; }
        if (strcasestr_safe(buf, "get_scale"))    { console_dispatch_short("get_scale");    return; }
        {   /* set_scale needs its numeric argument */
            const char *p = strcasestr_safe(buf, "set_scale");
            if (p != NULL) { console_dispatch_set_scale(p + 9); return; }
        }
        if (strcasestr_safe(buf, "selftest"))     { console_dispatch_short("selftest");     return; }
        if (strcasestr_safe(buf, "version"))      { console_dispatch_short("version");      return; }
        if (strcasestr_safe(buf, "unlock"))       { console_dispatch_short("unlock");       return; }

        /* --- SHORT single-word commands --- */
        if (strcasestr_safe(buf, "status"))       { console_dispatch_short("status");       return; }
        if (strcasestr_safe(buf, "close"))        { console_dispatch_short("close");        return; }
        if (strcasestr_safe(buf, "estop"))        { console_dispatch_short("estop");        return; }
        if (strcasestr_safe(buf, "weigh"))        { console_dispatch_short("weigh");        return; }
        if (strcasestr_safe(buf, "diag"))         { console_dispatch_short("diag");         return; }
        if (strcasestr_safe(buf, "home"))         { console_dispatch_short("home");         return; }
        if (strcasestr_safe(buf, "open"))         { console_dispatch_short("open");         return; }
        if (strcasestr_safe(buf, "halt"))         { console_dispatch_short("halt");         return; }
        if (strcasestr_safe(buf, "tare"))         { console_dispatch_short("tare");         return; }
        if (strcasestr_safe(buf, "lock"))         { console_dispatch_short("lock");         return; }
        if (strcasestr_safe(buf, "nop"))          { console_dispatch_short("nop");          return; }

        /* Nothing recognised — echo back as a warning so the user sees why */
        {
            char warn[160];
            int n = snprintf(warn, sizeof(warn),
                "\r\n[console] unknown command: '%s' — type 'help' for list\r\n",
                buf);
            if (n > 0 && n < (int)sizeof(warn))
                console_write_all(warn, (size_t)n);
        }
    }
}

/* ----------------------------------------------------------------------------
 *  console_task — line-buffered reader.
 *
 *  idf.py monitor (and most terminals) send each keystroke as it is typed,
 *  so a single read usually returns one character. Bytes are accumulated
 *  into a line buffer (with local echo + backspace) and the full line is
 *  handed to console_handle_line only on Enter (CR or LF).
 * -------------------------------------------------------------------------- */
static void console_task(void *arg) {
    (void)arg;
    uint8_t data[CONSOLE_LINE_MAX_IN + 1];
    static char line[CONSOLE_LINE_MAX_IN + 1];
    size_t line_len = 0;
    bool overflow = false;

    if (g_console_ready) uart_flush(UART_CONSOLE);

    const char *welcome =
        "\r\n--- sediment tray console ready (fw " FW_VERSION_STR ") ---\r\n"
        "    type 'help' for the full command list\r\n";
    console_write_all(welcome, strlen(welcome));

    for (;;) {
        int len = console_read_any(data, sizeof(data));
        if (len <= 0) continue;

        for (int i = 0; i < len; i++) {
            char c = (char)data[i];

            if (c == '\r' || c == '\n') {
                if (line_len == 0 && !overflow) continue;   /* swallow CRLF pair / empty Enter */
                console_write_all("\r\n", 2);
                if (overflow) {
                    const char *w = "[console] line too long — dropped\r\n";
                    console_write_all(w, strlen(w));
                } else {
                    /* trim leading / trailing whitespace */
                    while (line_len > 0 && (line[line_len - 1] == ' ' || line[line_len - 1] == '\t'))
                        line_len--;
                    line[line_len] = '\0';
                    char *start = line;
                    while (*start == ' ' || *start == '\t') start++;
                    if (*start) console_handle_line(start);
                }
                line_len = 0;
                overflow = false;
                continue;
            }

            if (c == 0x08 || c == 0x7F) {                   /* backspace / DEL */
                if (line_len > 0) {
                    line_len--;
                    console_write_all("\b \b", 3);
                }
                continue;
            }

            if ((unsigned char)c < 0x20) continue;          /* ignore other control chars */

            if (line_len < CONSOLE_LINE_MAX_IN) {
                line[line_len++] = c;
                console_write_all(&c, 1);                   /* local echo */
            } else {
                overflow = true;
            }
        }
    }
}

/*  ============================================================================
 *   SECTION 25  ·  weight_task     P1 · core 1 · 20 ms
 *  ========================================================================= */

static void weight_task(void *arg) {
    (void)arg;
    int32_t stable_ref = 0;

    for (;;) {
        if (gpio_get_level(PIN_HX711_DOUT) == 0) {
            int32_t r;
            xSemaphoreTake(g_lc_mutex, portMAX_DELAY);
            bool ok = hx711_read_blocking(&r, HX711_READ_TIMEOUT_US);
            if (ok) {
                loadcell_push_sample(r);
                int32_t avg = loadcell_current_avg();
                g_lc_last_avg = avg;
                float w = loadcell_counts_to_grams(avg);
                if (isnan(w) || isinf(w)) {
                    g_weight_ok = false;
                    fault_raise(FAULT_SENSOR_NAN, "nan_inf", 0, 0);
                } else {
                    g_weight_g  = w;
                    g_weight_ok = true;
                }
            }
            xSemaphoreGive(g_lc_mutex);

            int32_t cur = g_lc_last_avg;
            float drift = fabsf((float)(cur - stable_ref) /
                                (float)(g_lc_scale_cpg > 0 ? g_lc_scale_cpg : 1));
            if (drift <= WEIGHT_STABLE_EPSILON_G) {
                if ((ms_now() - g_weight_stable_since_ms) >= WEIGHT_STABLE_WIN_MS) {
                    g_weight_stable = true;
                }
            } else {
                g_weight_stable          = false;
                g_weight_stable_since_ms = ms_now();
                stable_ref               = cur;
            }

            if (g_weight_stable && g_weight_g > MAX_WEIGHT_G && !g_fault_latched) {
                fault_raise(FAULT_OVERWEIGHT, "stable_over",
                            (int32_t)g_weight_g, 0);
            }
        }
        vTaskDelay_ms(WEIGHT_PERIOD_MS);
    }
}

/*  ============================================================================
 *   SECTION 26  ·  app_main   —   boot sequence per flowchart v3.1.8
 *  ========================================================================= */

static esp_err_t uart_init_all(void) {
    const uart_config_t cfg_common = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_JSON, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_JSON, &cfg_common));
    ESP_ERROR_CHECK(uart_set_pin(UART_JSON, PIN_UART1_TX, PIN_UART1_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(UART_SERVO, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_SERVO, &cfg_common));
    ESP_ERROR_CHECK(uart_set_pin(UART_SERVO, PIN_UART2_TX, PIN_UART2_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    /* UART0 console — EXACT copy of grabber v1.28 uart_init() pattern:
     * dedicated config struct (no source_clk field — grabber doesn't set
     * one), install + param_config, no uart_set_pin, no error gating.   */
    const uart_config_t uart0_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(UART_CONSOLE, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_CONSOLE, &uart0_config);
    g_console_ready = true;

    /* Native USB-Serial-JTAG (/dev/ttyACM0) — the sdkconfig console sends
     * logs there as output-only, so install the driver to accept typed
     * commands from idf.py monitor on that port too.                    */
    usb_serial_jtag_driver_config_t usbj_cfg = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 1024,
    };
    if (usb_serial_jtag_driver_install(&usbj_cfg) == ESP_OK) {
        g_usbjtag_ready = true;
    }
    return ESP_OK;
}

static void raise_boot_fault_from_reset_cause(void) {
    switch (g_reset_cause) {
        case ESP_RST_BROWNOUT:
            fault_raise(FAULT_BROWNOUT, "boot", 0, 0);
            break;
        case ESP_RST_TASK_WDT:
        case ESP_RST_INT_WDT:
        case ESP_RST_WDT:
            /* Already recovered via reboot — log only, don't latch. */
            ESP_LOGW(TAG, "boot after WDT reset (reset_cause=%d) — not latched",
                     (int)g_reset_cause);
            break;
        default:
            break;
    }
}

void app_main(void) {
    g_mode        = MODE_BOOT;
    g_reset_cause = esp_reset_reason();

    gpio_setup();

    esp_err_t nerr = nvs_flash_init();
    if (nerr == ESP_ERR_NVS_NO_FREE_PAGES || nerr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(nerr);
    }
    if (nvs_restore_calibration() != ESP_OK) {
        ESP_LOGW(TAG, "NVS restore failed; running with defaults");
    }

    ESP_ERROR_CHECK(uart_init_all());
    /* uart_init_all now configures UART0/UART1/UART2 together.
     * g_console_ready is set inside uart_init_all on UART0 success,
     * so servo_detected and every subsequent uart1_send_json frame
     * are already mirrored to idf.py monitor by this point.          */

    int attempts = 0;
    esp_err_t ping = ESP_FAIL;
    for (int i = 0; i < 5 && ping != ESP_OK; i++) {
        attempts++;
        ping = servo_ping(SERVO_ID);
        if (ping != ESP_OK) vTaskDelay_ms(50);
    }
    if (ping == ESP_OK) {
        /* Human-readable notice in the ESP log → shows in idf.py monitor */
        ESP_LOGI(TAG, "motor response detected (servo id 0x%02X, attempts=%d)",
                 SERVO_ID, attempts);
        /* Machine-readable event to the host (and mirrored to UART0) */
        uart1_send_json("{\"event\":\"servo_detected\",\"seq\":0,"
                        "\"detected\":true,\"id\":%u,\"attempts\":%d"
                        TRAILER_FMT "}",
                        (unsigned)SERVO_ID, attempts, TRAILER_ARGS);
        servo_configure_protection(SERVO_ID);
        (void)servo_set_torque(SERVO_ID, false);
        g_motor_bus_ready = true;
    } else {
        ESP_LOGE(TAG, "servo_ping failed after %d attempts — motor bus not ready",
                 attempts);
        uart1_send_json("{\"event\":\"servo_detected\",\"seq\":0,"
                        "\"detected\":false,\"id\":%u,\"attempts\":%d"
                        TRAILER_FMT "}",
                        (unsigned)SERVO_ID, attempts, TRAILER_ARGS);
    }

    g_servo_cmd_q = xQueueCreate(SERVO_CMD_Q_DEPTH, sizeof(srv_cmd_t));
    if (!g_servo_cmd_q) { ESP_LOGE(TAG, "xQueueCreate failed"); esp_restart(); }

    g_lc_mutex = xSemaphoreCreateMutex();
    if (!g_lc_mutex) { ESP_LOGE(TAG, "mutex create failed"); esp_restart(); }

    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms     = TASK_WDT_TIMEOUT_MS * 10,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    esp_err_t werr = esp_task_wdt_reconfigure(&wdt_cfg);
    if (werr == ESP_ERR_INVALID_STATE) {
        /* already init'd by framework */
    } else if (werr != ESP_OK) {
        ESP_ERROR_CHECK(esp_task_wdt_init(&wdt_cfg));
    }

    nvs_update_boot_count();
    raise_boot_fault_from_reset_cause();
    emit_boot_event();

    g_mode = MODE_IDLE;

    /* UART0 console driver was installed earlier (before servo_ping emission)
     * — console_task now gets spawned alongside the other tasks below.     */

    xTaskCreatePinnedToCore(safety_task,   "safety",   4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(status_task,   "status",   6144, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(json_cmd_task, "json_cmd", 6144, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(weight_task,   "weight",   4096, NULL, 1, NULL, 1);
    /* Priority 6 — same as grabber's motor_task that this is modelled on.
     * Not pinned (grabber uses plain xTaskCreate).                        */
    xTaskCreate(console_task, "console", 4096, NULL, 2, NULL);

    ESP_LOGI(TAG, "Sediment Tray FW v" FW_VERSION_STR " booted — boots=%" PRIu32
             " reset_cause=%d", g_boot_count, (int)g_reset_cause);
}

/*  ============================================================================
 *                                END OF FILE
 *  ========================================================================= */
