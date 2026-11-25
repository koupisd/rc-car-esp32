/*
 * ============================================================
 *  RC Car Firmware — ESP32 + DRV8833 + 4× TT Motors + PS3
 *                  + MAX98357A I2S Amplifier (engine sound + horn)
 *  Uses: PS3 Controller Host library (jvpernis)
 *        Requires ESP32 Arduino core v2.x.x (NOT v3.x)
 * ============================================================
 *
 *  ── SETUP INSTRUCTIONS ────────────────────────────────────
 *
 *  Step 1 — Install ESP32 core v2.x.x:
 *    Tools → Board → Boards Manager → "ESP32 by Espressif"
 *    Select version 2.0.17 and install.
 *
 *  Step 2 — Install PS3 Controller Host library:
 *    Sketch → Include Library → Manage Libraries
 *    Search "PS3 Controller Host" by Jeffrey van Pernis
 *    Install it.
 *
 *  Step 3 — Find your ESP32 Bluetooth MAC:
 *    Upload the MAC-finder sketch, note the address printed
 *    on Serial Monitor (e.g. C4:DE:E2:19:66:26).
 *
 *  Step 4 — Flash the MAC into the PS3 controller:
 *    Connect controller to PC via Mini-USB cable.
 *    Open SixaxisPairTool, enter the ESP32 MAC, click Update.
 *
 *  Step 5 — Upload this sketch, press the PS button.
 *    The controller LEDs will flash then settle on LED 1.
 *
 *  ── HARDWARE ──────────────────────────────────────────────
 *    DRV8833 #1 → Left Front (LF) + Right Front (RF) motors
 *    DRV8833 #2 → Left Rear  (LR) + Right Rear  (RR) motors
 *    Connect nSLEEP on both DRV8833s to 3.3V (keep awake)
 *    Common GND between ESP32, both DRV8833s, and battery
 *
 *  ── MAX98357A WIRING ──────────────────────────────────────
 *    MAX98357A LRC  (WS)   → GPIO 23
 *    MAX98357A BCLK        → GPIO 13
 *    MAX98357A DIN         → GPIO 15
 *    MAX98357A SD (mute)   → GPIO 27  (optional; tie to VIN to skip)
 *    MAX98357A VIN         → 5 V
 *    MAX98357A GND         → GND
 *
 *    NOTE: The original active-buzzer horn on GPIO 22 is removed.
 *          The Cross × button now triggers the synthesized horn
 *          through the MAX98357A speaker instead.
 *
 *  ── PS3 BUTTON MAPPING ────────────────────────────────────
 *    Left  stick Y  → Throttle (forward/reverse)
 *    Right stick X  → Steering (differential front wheels)
 *    R2             → Turbo (hold to unlock full speed)
 *    Cross  ×       → Horn (hold) — via MAX98357A
 *    Triangle ▲     → Headlights toggle
 *    Square  ■      → Hazard lights toggle
 *    L1             → Left indicator toggle
 *    R1             → Right indicator toggle
 * ============================================================
 */

#include <Ps3Controller.h>
#include <driver/i2s.h>
#include <math.h>

// ─────────────────────────────────────────────
//  !! PASTE YOUR ESP32 BLUETOOTH MAC HERE !!
// ─────────────────────────────────────────────
#define ESP32_BT_MAC   "C4:DE:E2:19:66:26"

// ─────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────

// DRV8833 #1 — Front axle
#define LF_IN1   14
#define LF_IN2   12
#define RF_IN1   26
#define RF_IN2   25

// DRV8833 #2 — Rear axle
#define LR_IN1   32
#define LR_IN2   33
#define RR_IN1   18
#define RR_IN2   19

// Lights
#define PIN_HEADLIGHTS   4
#define PIN_TAILLIGHTS   5
#define PIN_REV_LIGHTS   16
#define PIN_IND_LEFT     17
#define PIN_IND_RIGHT    21

// MAX98357A I2S pins  (chosen to avoid all motor + lights pins above)
#define I2S_WS_PIN    23   // LRC
#define I2S_BCK_PIN   13   // BCLK
#define I2S_DATA_PIN  15   // DIN
#define I2S_SD_PIN    27   // MAX98357A SD (HIGH=on, LOW=mute). Set to -1 to disable.

// ─────────────────────────────────────────────
//  PWM  (ESP32 core v2.x API)
// ─────────────────────────────────────────────
#define PWM_FREQ   1000
#define PWM_RES    8        // 0–255

#define CH_LF_IN1   0
#define CH_LF_IN2   1
#define CH_RF_IN1   2
#define CH_RF_IN2   3
#define CH_LR_IN1   4
#define CH_LR_IN2   5
#define CH_RR_IN1   6
#define CH_RR_IN2   7
// Channels 8–15 are free; I2S does not use LEDC channels.

// ─────────────────────────────────────────────
//  TUNING
// ─────────────────────────────────────────────
#define DEADZONE             10
#define NORMAL_SPEED_LIMIT   180
#define TURBO_SPEED          255
#define STEERING_STRENGTH    0.6f
#define INDICATOR_PERIOD_MS  500

// ─────────────────────────────────────────────
//  AUDIO CONSTANTS
// ─────────────────────────────────────────────
#define AUDIO_SAMPLE_RATE   22050
#define I2S_PORT            I2S_NUM_0
#define DMA_BUF_COUNT       8
#define DMA_BUF_LEN         256         // samples per DMA buffer

// Engine: fundamental Hz at idle and full throttle
#define ENGINE_FREQ_IDLE    60.0f
#define ENGINE_FREQ_MAX    180.0f
#define ENGINE_MASTER_VOL   0.70f       // 0.0–1.0

// Harmonics table: { frequency_multiplier, relative_amplitude }
// Layering these gives a diesel-like growl rather than a pure sine.
static const float HARMONICS[][2] = {
    { 1.0f, 1.00f },
    { 2.0f, 0.55f },
    { 3.0f, 0.30f },
    { 4.0f, 0.15f },
    { 5.0f, 0.08f },
};
#define NUM_HARMONICS  (sizeof(HARMONICS) / sizeof(HARMONICS[0]))

// Horn: dual-tone A4 + C#5 (classic two-note car horn)
#define HORN_FREQ1      440.0f
#define HORN_FREQ2      554.4f
#define HORN_VOL        0.70f

#define TWO_PI_F        6.28318530f

// ─────────────────────────────────────────────
//  STATE
// ─────────────────────────────────────────────
bool headlightsOn = false;
bool hazardOn     = false;
bool indLeftOn    = false;
bool indRightOn   = false;

bool prevTriangle = false;
bool prevSquare   = false;
bool prevL1       = false;
bool prevR1       = false;

unsigned long lastBlink  = 0;
bool          blinkState = false;

// Audio state — written from Core 1 (loop), read from Core 0 (audio task).
// 32-bit aligned scalar writes are atomic on Xtensa; no mutex needed.
volatile float s_throttle     = 0.0f;   // 0.0 = idle, 1.0 = full
volatile bool  s_horn_active  = false;

// Per-harmonic phase accumulators (only touched inside audio task on Core 0)
static float s_eng_phase[NUM_HARMONICS] = { 0 };
static float s_horn_phase1 = 0.0f;
static float s_horn_phase2 = 0.0f;

static TaskHandle_t s_audio_task = NULL;

// ═════════════════════════════════════════════
//  AUDIO — SYNTHESIS + I2S TASK
// ═════════════════════════════════════════════

static void fillAudioBuffer(int16_t* buf, int n) {
    float throttle = s_throttle;
    bool  horn     = s_horn_active;

    float engFreq   = ENGINE_FREQ_IDLE + throttle * (ENGINE_FREQ_MAX - ENGINE_FREQ_IDLE);
    float engVol    = ENGINE_MASTER_VOL * (0.5f + 0.5f * throttle); // quieter at idle

    // Pre-calculate per-harmonic phase increments
    float phInc[NUM_HARMONICS];
    for (int h = 0; h < (int)NUM_HARMONICS; h++) {
        phInc[h] = TWO_PI_F * engFreq * HARMONICS[h][0] / AUDIO_SAMPLE_RATE;
    }
    float hornInc1 = TWO_PI_F * HORN_FREQ1 / AUDIO_SAMPLE_RATE;
    float hornInc2 = TWO_PI_F * HORN_FREQ2 / AUDIO_SAMPLE_RATE;

    for (int i = 0; i < n; i++) {
        float sample = 0.0f;

        // ── Engine harmonics ──────────────────────────────────────────
        float eng = 0.0f;
        for (int h = 0; h < (int)NUM_HARMONICS; h++) {
            eng += sinf(s_eng_phase[h]) * HARMONICS[h][1];
            s_eng_phase[h] += phInc[h];
            if (s_eng_phase[h] > TWO_PI_F) s_eng_phase[h] -= TWO_PI_F;
        }
        sample += eng * engVol;

        // ── Horn (dual-tone) ──────────────────────────────────────────
        if (horn) {
            float h = sinf(s_horn_phase1) + sinf(s_horn_phase2);
            sample += h * (HORN_VOL * 0.5f);
            s_horn_phase1 += hornInc1;
            s_horn_phase2 += hornInc2;
            if (s_horn_phase1 > TWO_PI_F) s_horn_phase1 -= TWO_PI_F;
            if (s_horn_phase2 > TWO_PI_F) s_horn_phase2 -= TWO_PI_F;
        }

        // ── Clamp + convert ───────────────────────────────────────────
        if (sample >  1.0f) sample =  1.0f;
        if (sample < -1.0f) sample = -1.0f;
        buf[i] = (int16_t)(sample * 32767.0f);
    }
}

static void audioTask(void* pv) {
    static int16_t buf[DMA_BUF_LEN];
    size_t written;
    for (;;) {
        fillAudioBuffer(buf, DMA_BUF_LEN);
        i2s_write(I2S_PORT, buf, DMA_BUF_LEN * sizeof(int16_t), &written, portMAX_DELAY);
    }
}

void audioBegin() {
    // Bring MAX98357A out of shutdown
#if I2S_SD_PIN >= 0
    pinMode(I2S_SD_PIN, OUTPUT);
    digitalWrite(I2S_SD_PIN, HIGH);
#endif

    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate          = AUDIO_SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,   // mono amp
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = DMA_BUF_COUNT,
        .dma_buf_len          = DMA_BUF_LEN,
        .use_apll             = false,
        .tx_desc_auto_clear   = true,
    };
    i2s_pin_config_t pins = {
        .bck_io_num   = I2S_BCK_PIN,
        .ws_io_num    = I2S_WS_PIN,
        .data_out_num = I2S_DATA_PIN,
        .data_in_num  = I2S_PIN_NO_CHANGE,
    };
    i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
    i2s_set_pin(I2S_PORT, &pins);
    i2s_zero_dma_buffer(I2S_PORT);

    // Run audio synthesis on Core 0; PS3 + motor logic stays on Core 1
    xTaskCreatePinnedToCore(
        audioTask,
        "audio",
        4096,
        NULL,
        configMAX_PRIORITIES - 1,   // highest priority — DMA must never starve
        &s_audio_task,
        0   // Core 0
    );
}

// ═════════════════════════════════════════════
//  MOTOR FUNCTIONS
// ═════════════════════════════════════════════

void pwmSetup(uint8_t channel, uint8_t pin) {
    ledcSetup(channel, PWM_FREQ, PWM_RES);
    ledcAttachPin(pin, channel);
}

void driveMotor(uint8_t chIN1, uint8_t chIN2, int speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        ledcWrite(chIN1, (uint32_t)speed);
        ledcWrite(chIN2, 0);
    } else if (speed < 0) {
        ledcWrite(chIN1, 0);
        ledcWrite(chIN2, (uint32_t)(-speed));
    } else {
        ledcWrite(chIN1, 0);
        ledcWrite(chIN2, 0);   // coast
    }
}

void setLF(int s) { driveMotor(CH_LF_IN1, CH_LF_IN2, s); }
void setRF(int s) { driveMotor(CH_RF_IN1, CH_RF_IN2, s); }
void setLR(int s) { driveMotor(CH_LR_IN1, CH_LR_IN2, s); }
void setRR(int s) { driveMotor(CH_RR_IN1, CH_RR_IN2, s); }
void stopAll()    { setLF(0); setRF(0); setLR(0); setRR(0); }

// ═════════════════════════════════════════════
//  DIFFERENTIAL STEERING
// ═════════════════════════════════════════════
void applyDrive(int throttle, int steering) {
    setLR(throttle);
    setRR(throttle);

    float steerNorm = steering / 255.0f;
    float outer     = (float)throttle;
    float inner     = outer * (1.0f - fabsf(steerNorm) * STEERING_STRENGTH);

    int lf, rf;
    if (steering < 0) {
        lf = (int)inner;
        rf = (int)outer;
    } else if (steering > 0) {
        lf = (int)outer;
        rf = (int)inner;
    } else {
        lf = rf = throttle;
    }

    setLF(constrain(lf, -255, 255));
    setRF(constrain(rf, -255, 255));
}

// ═════════════════════════════════════════════
//  LIGHTS
// ═════════════════════════════════════════════
void updateLights(bool reversing) {
    digitalWrite(PIN_HEADLIGHTS, headlightsOn ? HIGH : LOW);
    digitalWrite(PIN_TAILLIGHTS, headlightsOn ? HIGH : LOW);
    digitalWrite(PIN_REV_LIGHTS, reversing    ? HIGH : LOW);
}

void updateIndicators() {
    unsigned long now = millis();
    if (now - lastBlink >= INDICATOR_PERIOD_MS) {
        lastBlink  = now;
        blinkState = !blinkState;
    }
    bool showL = false, showR = false;
    if (hazardOn) {
        showL = showR = blinkState;
    } else {
        if (indLeftOn)  showL = blinkState;
        if (indRightOn) showR = blinkState;
    }
    digitalWrite(PIN_IND_LEFT,  showL ? HIGH : LOW);
    digitalWrite(PIN_IND_RIGHT, showR ? HIGH : LOW);
}

// ═════════════════════════════════════════════
//  PS3 CALLBACKS
// ═════════════════════════════════════════════
void onConnect() {
    Serial.println("[PS3] Controller connected!");
    Ps3.setPlayer(1);
}

void onDisconnect() {
    Serial.println("[PS3] Controller disconnected — motors stopped.");
    stopAll();
    s_throttle    = 0.0f;   // engine drops to idle
    s_horn_active = false;
}

// ═════════════════════════════════════════════
//  PROCESS CONTROLLER INPUT
// ═════════════════════════════════════════════
void processController() {

    // ── Axes ──────────────────────────────────────────────────────────
    int rawThrottle = Ps3.data.analog.stick.ly;
    int rawSteering = Ps3.data.analog.stick.rx;

    int throttle = map(-rawThrottle, -128, 127, -255, 255);
    int steering = map( rawSteering, -128, 127, -255, 255);

    if (abs(throttle) < DEADZONE * 2) throttle = 0;
    if (abs(steering) < DEADZONE * 2) steering = 0;

    // ── Turbo (R2) ────────────────────────────────────────────────────
    bool turbo   = (Ps3.data.analog.button.r2 > 30);
    int speedCap = turbo ? TURBO_SPEED : NORMAL_SPEED_LIMIT;
    throttle = constrain(throttle, -speedCap, speedCap);

    // ── Drive ─────────────────────────────────────────────────────────
    applyDrive(throttle, steering);
    bool reversing = (throttle < 0);

    // ── Audio — engine rev ────────────────────────────────────────────
    // Map |throttle| (0–speedCap) → 0.0–1.0 for the synthesiser.
    // Engine idles even when the car is stationary.
    s_throttle = (float)abs(throttle) / (float)TURBO_SPEED;

    // ── Toggle buttons ────────────────────────────────────────────────

    // Triangle ▲ — Headlights
    bool curTriangle = Ps3.data.button.triangle;
    if (curTriangle && !prevTriangle) {
        headlightsOn = !headlightsOn;
        Serial.printf("Headlights: %s\n", headlightsOn ? "ON" : "OFF");
    }
    prevTriangle = curTriangle;

    // Square ■ — Hazard
    bool curSquare = Ps3.data.button.square;
    if (curSquare && !prevSquare) {
        hazardOn = !hazardOn;
        if (hazardOn) { indLeftOn = false; indRightOn = false; }
        Serial.printf("Hazard: %s\n", hazardOn ? "ON" : "OFF");
    }
    prevSquare = curSquare;

    // L1 — Left indicator
    bool curL1 = Ps3.data.button.l1;
    if (curL1 && !prevL1) {
        indLeftOn  = !indLeftOn;
        indRightOn = false;
        hazardOn   = false;
        Serial.printf("Left indicator: %s\n", indLeftOn ? "ON" : "OFF");
    }
    prevL1 = curL1;

    // R1 — Right indicator
    bool curR1 = Ps3.data.button.r1;
    if (curR1 && !prevR1) {
        indRightOn = !indRightOn;
        indLeftOn  = false;
        hazardOn   = false;
        Serial.printf("Right indicator: %s\n", indRightOn ? "ON" : "OFF");
    }
    prevR1 = curR1;

    // Cross × — Horn (hold to sound, via MAX98357A)
    s_horn_active = Ps3.data.button.cross;

    // ── Lights ────────────────────────────────────────────────────────
    updateLights(reversing);

    // ── Debug (un-comment if needed) ──────────────────────────────────
    /*
    Serial.printf("THR:%4d  STR:%4d  TURBO:%d  HEAD:%d  HAZ:%d  L:%d  R:%d\n",
        throttle, steering, (int)turbo,
        (int)headlightsOn, (int)hazardOn,
        (int)indLeftOn, (int)indRightOn);
    */
}

// ═════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    Serial.println("=== RC Car ESP32 + PS3 + MAX98357A Audio ===");

    // Motor PWM
    pwmSetup(CH_LF_IN1, LF_IN1); pwmSetup(CH_LF_IN2, LF_IN2);
    pwmSetup(CH_RF_IN1, RF_IN1); pwmSetup(CH_RF_IN2, RF_IN2);
    pwmSetup(CH_LR_IN1, LR_IN1); pwmSetup(CH_LR_IN2, LR_IN2);
    pwmSetup(CH_RR_IN1, RR_IN1); pwmSetup(CH_RR_IN2, RR_IN2);
    stopAll();

    // Lights (PIN_HORN removed — horn is now via MAX98357A)
    pinMode(PIN_HEADLIGHTS, OUTPUT);
    pinMode(PIN_TAILLIGHTS, OUTPUT);
    pinMode(PIN_REV_LIGHTS, OUTPUT);
    pinMode(PIN_IND_LEFT,   OUTPUT);
    pinMode(PIN_IND_RIGHT,  OUTPUT);
    digitalWrite(PIN_HEADLIGHTS, LOW);
    digitalWrite(PIN_TAILLIGHTS, LOW);
    digitalWrite(PIN_REV_LIGHTS, LOW);
    digitalWrite(PIN_IND_LEFT,   LOW);
    digitalWrite(PIN_IND_RIGHT,  LOW);

    // PS3 — must come before audioBegin() so BT stack is up first
    Ps3.attach(nullptr);
    Ps3.attachOnConnect(onConnect);
    Ps3.attachOnDisconnect(onDisconnect);
    Ps3.begin(ESP32_BT_MAC);
    Serial.println("Waiting for PS3 controller — press the PS button.");

    // Audio — I2S task on Core 0, started after BT init
    audioBegin();
    Serial.println("Audio ready.");
}

// ═════════════════════════════════════════════
//  MAIN LOOP
// ═════════════════════════════════════════════
void loop() {
    if (Ps3.isConnected()) {
        processController();
    } else {
        stopAll();
        s_throttle    = 0.0f;   // keep engine at idle when no controller
        s_horn_active = false;
    }

    updateIndicators();

    delay(10);   // ~100 Hz
}
