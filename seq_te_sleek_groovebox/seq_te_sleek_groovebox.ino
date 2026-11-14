#include <driver/i2s.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <math.h>

// ==========================================
// 0. FIRMWARE VERSION
// ==========================================
#define FW_NAME    "GROOVEBOX"
#define FW_VERSION "v5.1"
// v5.1: FN layer (long-press TRACK), Track Mute, Step Probability,
//       Per-Step Note, Delay Mix, Play/Pause, and unified OLED UI.
// v5.0: 32-bit APLL I2S, PolyBLEP oscillators, SVF resonant filter,
//       master DC blocker, and BPM-synced metronome screensaver.

// ==========================================
// 1. PIN CONFIGURATIONS (100% UNCHANGED)
// ==========================================

// I2S Audio to PCM5102A DAC
#define I2S_BCK 26
#define I2S_WS 25
#define I2S_DOUT 27
#define SAMPLE_RATE 44100
#define BUFFER_SIZE 128

// OLED Display (SH1106 SPI)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_DC 16
#define OLED_CS 4
#define OLED_RESET 17
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS);

// Potentiometers / Encoders
#define ENC_NAV_CLK 34
#define ENC_NAV_DT 35
#define ENC_PITCH_CLK 5     // Semitone Transpose (-12 to +12)
#define ENC_PITCH_DT 13
#define ENC_DECAY_CLK 33    // Note Decay / Tightness (50ms to 600ms)
#define ENC_DECAY_DT 14
#define ENC_TEMPO_CLK 21    // BPM (60 to 200)
#define ENC_TEMPO_DT 19

// Buttons & LED
#define BTN_TOGGLE 22
#define BTN_TRACK 32
#define LED_BEAT 2

// ==========================================
// 2. SEQUENCER & SYNTH PARAMETERS
// ==========================================
#define NUM_TRACKS 8
#define NUM_STEPS 16

const char* trackNames[] = {
  "1: KICK", "2: HI-HAT", "3: SUB-BASS", "4: SAW-BASS",
  "5: PLUCK", "6: LEAD",   "7: STAB",     "8: CHIME"
};
const char* waveNames[] = {"KCK", "HAT", "SUB", "SAW", "PLK", "STB"};
const char* keyNames[]  = {"A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#"};

struct Track {
  bool active;
  float phase;
  float baseFreq;
  float currentFreq;
  int waveType;
  float pan;
  float baseVolume;
  float currentVolume;
  float envLevel;
  int envStage;
  int envCounter;
  int attackSamples;
  int baseDecaySamples;
  int currentDecaySamples;
  float filterState1;
  float filterState2;
};

Track tracks[NUM_TRACKS];
bool sequence[NUM_TRACKS][NUM_STEPS] = {0};
uint8_t stepProb[NUM_TRACKS][NUM_STEPS];
int8_t stepNote[NUM_TRACKS][NUM_STEPS] = {0};
bool trackMuted[NUM_TRACKS] = {false};

int currentTrack = 0;
int currentStep = 0;
int playbackStep = 0;
int tempo = 120;
int semitoneOffset = 0;
int globalDecayMs = 160;
float delayMix = 0.40f;
bool fnMode = false;
bool isPlaying = true;

// MPC-Style Swing (58% shuffle - active in background)
float swingPercent = 0.58f;
float sidechainAmount = 0.0f;

// 3D Stereo Ping-Pong Delay Line
#define DELAY_BUF_SIZE 16384
#define DELAY_MASK (DELAY_BUF_SIZE - 1)
int16_t delayBufferL[DELAY_BUF_SIZE] = {0};
int16_t delayBufferR[DELAY_BUF_SIZE] = {0};
int delayWriteIndex = 0;
int delayOffset = 11025;
float delayFilterL = 0.0f;
float delayFilterR = 0.0f;

unsigned long lastStepTime = 0;
unsigned long currentStepDuration = 0;
unsigned long ledOnTime = 0;
unsigned long lastNavTime = 0;

// Screensaver timer (triggers after 30s of inactivity)
unsigned long lastUserInteraction = 0;

int lastNavCLK = HIGH, lastPitchCLK = HIGH, lastDecayCLK = HIGH, lastTempoCLK = HIGH;
int lastToggleBtn = HIGH, lastTrackBtn = HIGH;
unsigned long trackBtnDownTime = 0;
bool trackLongHandled = false;
unsigned long toggleBtnDownTime = 0;
bool toggleLongHandled = false;
uint32_t noiseSeed = 12345;

void resetScreensaverTimer() {
  lastUserInteraction = millis();
}

void updateTrackFrequencies() {
  float ratio = powf(2.0f, (float)semitoneOffset / 12.0f);
  float kickRatio = powf(2.0f, (float)semitoneOffset * 0.25f / 12.0f);

  tracks[0].baseFreq = 48.0f * kickRatio;
  tracks[1].baseFreq = 7000.0f;
  tracks[2].baseFreq = 55.0f  * ratio;
  tracks[3].baseFreq = 110.0f * ratio;
  tracks[4].baseFreq = 220.0f * ratio;
  tracks[5].baseFreq = 330.0f * ratio;
  tracks[6].baseFreq = 440.0f * ratio;
  tracks[7].baseFreq = 660.0f * ratio;
}

void updateTrackDecayTimes() {
  int baseDecaySamples = globalDecayMs * 44;

  tracks[0].baseDecaySamples = constrain((int)(baseDecaySamples * 1.2f), 3000, 15000);
  tracks[1].baseDecaySamples = constrain((int)(baseDecaySamples * 0.28f), 800, 2500);
  tracks[2].baseDecaySamples = (int)(baseDecaySamples * 1.35f);
  tracks[3].baseDecaySamples = (int)(baseDecaySamples * 1.0f);
  tracks[4].baseDecaySamples = (int)(baseDecaySamples * 0.9f);
  tracks[5].baseDecaySamples = (int)(baseDecaySamples * 0.65f);
  tracks[6].baseDecaySamples = (int)(baseDecaySamples * 0.85f);
  tracks[7].baseDecaySamples = (int)(baseDecaySamples * 0.75f);

  for (int i = 0; i < NUM_TRACKS; i++) {
    tracks[i].currentDecaySamples = tracks[i].baseDecaySamples;
  }
}

void initializeTracks() {
  tracks[0] = {false, 0, 48.0f,  48.0f,  0,  0.0f, 0.70f, 0.70f, 0, 0, 0,  40, 8000, 8000, 0, 0}; // KCK
  tracks[1] = {false, 0, 7000.0f,7000.0f,1,  0.20f,0.28f, 0.28f, 0, 0, 0,  15, 1500, 1500, 0, 0}; // HAT
  tracks[2] = {false, 0, 55.0f,  55.0f,  2,  0.0f, 0.65f, 0.65f, 0, 0, 0,  60, 9000, 9000, 0, 0}; // SUB
  tracks[3] = {false, 0, 110.0f, 110.0f, 3, -0.15f,0.45f, 0.45f, 0, 0, 0,  50, 7500, 7500, 0, 0}; // SAW BASS
  tracks[4] = {false, 0, 220.0f, 220.0f, 4, -0.35f,0.42f, 0.42f, 0, 0, 0,  20, 7000, 7000, 0, 0}; // PLK
  tracks[5] = {false, 0, 330.0f, 330.0f, 3,  0.30f,0.40f, 0.40f, 0, 0, 0,  60, 8000, 8000, 0, 0}; // LEAD
  tracks[6] = {false, 0, 440.0f, 440.0f, 5, -0.20f,0.38f, 0.38f, 0, 0, 0,  40, 6500, 6500, 0, 0}; // STB
  tracks[7] = {false, 0, 660.0f, 660.0f, 4,  0.35f,0.38f, 0.38f, 0, 0, 0,  20, 6000, 6000, 0, 0}; // CHIME

  updateTrackFrequencies();
  updateTrackDecayTimes();
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.print(FW_NAME);
  Serial.print(" ");
  Serial.print(FW_VERSION);
  Serial.print(" (built ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.print(__TIME__);
  Serial.println(")");

  // Large 16 x 256 DMA Buffer (~93ms) prevents audio dropouts
  // 32-bit I2S + APLL gives PCM5102A a low-jitter, studio-clean noise floor
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 16,
    .dma_buf_len = 256,
    .use_apll = true
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);

  initializeTracks();
  for (int t = 0; t < NUM_TRACKS; t++) {
    for (int s = 0; s < NUM_STEPS; s++) {
      stepProb[t][s] = 4; // 100% default
    }
  }
  memset(delayBufferL, 0, sizeof(delayBufferL));
  memset(delayBufferR, 0, sizeof(delayBufferR));

  SPI.begin();
  display.begin(0x3C, true);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(22, 16);
  display.print(FW_NAME);
  display.print(" ");
  display.println(FW_VERSION);
  display.setCursor(6, 32);
  display.println("FN Layer + Step Prob");
  display.setCursor(26, 46);
  display.println(__DATE__);
  display.display();
  delay(1200);

  pinMode(ENC_NAV_CLK, INPUT_PULLUP);   pinMode(ENC_NAV_DT, INPUT_PULLUP);
  pinMode(ENC_PITCH_CLK, INPUT_PULLUP); pinMode(ENC_PITCH_DT, INPUT_PULLUP);
  pinMode(ENC_DECAY_CLK, INPUT_PULLUP); pinMode(ENC_DECAY_DT, INPUT_PULLUP);
  pinMode(ENC_TEMPO_CLK, INPUT_PULLUP); pinMode(ENC_TEMPO_DT, INPUT_PULLUP);
  pinMode(BTN_TOGGLE, INPUT_PULLUP);    pinMode(BTN_TRACK, INPUT_PULLUP);
  pinMode(LED_BEAT, OUTPUT);

  unsigned long baseInterval = (60000 / tempo) / 4;
  currentStepDuration = (unsigned long)(baseInterval * 2.0f * swingPercent);
  delayOffset = constrain((int)((60000.0f / tempo) * 0.75f * (SAMPLE_RATE / 1000.0f)), 4000, DELAY_BUF_SIZE - 1);

  lastUserInteraction = millis();
}

void triggerTrack(int trackNum, int stepNum) {
  Track &t = tracks[trackNum];
  bool wasActive = t.active;
  t.active = true;
  if (!wasActive || t.waveType == 0) {
    t.phase = 0.0f;
    t.envLevel = 0.0f;
    t.envCounter = 0;
    t.filterState1 = 0.0f;
    t.filterState2 = 0.0f;
  } else {
    // Legato retrigger: avoid zero-crossing clicks & filter pops on consecutive steps
    t.envCounter = constrain((int)(t.envLevel * t.attackSamples), 0, t.attackSamples);
  }
  t.envStage = 1;

  // 1. Human Micro-Dynamics (Velocity Drift: subtle ±7% natural acoustic variation)
  float humanVelocity = 0.93f + ((float)(esp_random() % 100) / 100.0f) * 0.14f;
  t.currentVolume = t.baseVolume * humanVelocity;

  // 2. Micro-Decay Drift: subtle ±8% resonance variance per note
  float decayDrift = 0.92f + ((float)(esp_random() % 100) / 100.0f) * 0.16f;
  t.currentDecaySamples = (int)(t.baseDecaySamples * decayDrift);

  // 3. Per-step semitone offset (deterministic pitch; randomness is via Step Probability)
  int8_t noteOff = stepNote[trackNum][stepNum];
  float noteRatio = (noteOff == 0) ? 1.0f : powf(2.0f, (float)noteOff / 12.0f);

  if (t.waveType == 0) {
    t.currentFreq = 145.0f;
    sidechainAmount = 0.35f;
  } else {
    t.currentFreq = t.baseFreq * noteRatio;
  }
}

inline float softLimit(float x) {
  return x / sqrtf(1.0f + x * x);
}

// PolyBLEP anti-aliasing correction for crisp, alias-free oscillators
inline float polyBlep(float t, float dt) {
  if (t < dt) {
    t /= dt;
    return t + t - t * t - 1.0f;
  } else if (t > 1.0f - dt) {
    t = (t - 1.0f) / dt;
    return t * t + t + t + 1.0f;
  }
  return 0.0f;
}

// Chamberlin State-Variable Filter (mode: 0 = LP, 1 = HP, 2 = BP)
inline float svfProcess(float input, float &low, float &band, float f, float q, int mode) {
  float high = input - low - q * band;
  band += f * high;
  low += f * band;
  if (mode == 1) return high;
  if (mode == 2) return band;
  return low;
}

// Master DC-blocker state (~20 Hz high-pass)
float dcPrevInL = 0.0f, dcPrevOutL = 0.0f;
float dcPrevInR = 0.0f, dcPrevOutR = 0.0f;

// ==========================================
// 3. AUDIO ENGINE
// ==========================================
void generateAudio() {
  int32_t audioBuffer[BUFFER_SIZE * 2];

  for (int s = 0; s < BUFFER_SIZE; s++) {
    sidechainAmount *= 0.99965f;
    float duckMultiplier = 1.0f - sidechainAmount;

    float dryL = 0.0f, dryR = 0.0f;
    float delaySend = 0.0f;

    for (int i = 0; i < NUM_TRACKS; i++) {
      Track &t = tracks[i];
      if (!t.active) continue;

      if (t.envStage == 1) {
        t.envCounter++;
        t.envLevel = (float)t.envCounter / t.attackSamples;
        if (t.envCounter >= t.attackSamples) {
          t.envStage = 2;
          t.envCounter = 0;
          t.envLevel = 1.0f;
        }
      }
      else if (t.envStage == 2) {
        t.envCounter++;
        float progress = (float)t.envCounter / t.currentDecaySamples;
        t.envLevel = (1.0f - progress) * (1.0f - progress);
        if (t.envCounter >= t.currentDecaySamples || t.envLevel <= 0.002f) {
          t.active = false;
          t.envStage = 0;
          t.envLevel = 0.0f;
        }
      }

      float p = t.phase / (2.0f * M_PI);
      float dt = t.currentFreq / SAMPLE_RATE;
      float rawSample = 0.0f;

      if (t.waveType == 0) {
        // Pure 808 Kick with subtle 2ms transient click
        rawSample = sinf(t.phase);
        if (t.envStage == 1 || (t.envStage == 2 && t.envCounter < 90)) {
          float clickEnv = (t.envStage == 1) ? t.envLevel : (1.0f - (float)t.envCounter / 90.0f);
          rawSample += sinf(t.phase * 3.0f) * 0.25f * clickEnv;
        }
        t.currentFreq = t.baseFreq + (t.currentFreq - t.baseFreq) * 0.9991f;
      }
      else if (t.waveType == 1) {
        // Silky SVF High-Pass Hi-Hat
        noiseSeed = noiseSeed * 1664525 + 1013904223;
        float noise = ((float)(noiseSeed & 0xFFFF) / 32768.0f) - 1.0f;
        rawSample = svfProcess(noise, t.filterState1, t.filterState2, 0.72f, 0.85f, 1) * 0.85f;
      }
      else if (t.waveType == 2) {
        // Clean Sub-Bass with gentle harmonic warmth
        float fundamental = sinf(t.phase);
        float overtone1   = sinf(t.phase * 2.0f) * 0.22f;
        rawSample = svfProcess(fundamental + overtone1, t.filterState1, t.filterState2, 0.14f, 0.95f, 0);
      }
      else if (t.waveType == 3) {
        // PolyBLEP Saw + Resonant SVF Low-Pass with Envelope Sweep
        float saw = 2.0f * p - 1.0f - polyBlep(p, dt);
        float f = constrain(0.06f + 0.38f * t.envLevel * t.envLevel, 0.04f, 0.72f);
        rawSample = svfProcess(saw, t.filterState1, t.filterState2, f, 0.75f, 0) * 1.1f;
      }
      else if (t.waveType == 4) {
        // Pure Glassy FM Pluck / Chime (harmonic index decays with envelope)
        float modIndex = 1.4f * t.envLevel * t.envLevel;
        float modulator = sinf(t.phase * 2.0f) * modIndex;
        rawSample = sinf(t.phase + modulator) * 0.9f + sinf(t.phase * 3.0f) * 0.12f * t.envLevel;
      }
      else if (t.waveType == 5) {
        // Clean Stab: PolyBLEP Saw + Fundamental Sine through Resonant SVF
        float saw = 2.0f * p - 1.0f - polyBlep(p, dt);
        float mix = saw * 0.55f + sinf(t.phase) * 0.45f;
        float f = constrain(0.10f + 0.45f * t.envLevel * t.envLevel, 0.05f, 0.78f);
        rawSample = svfProcess(mix, t.filterState1, t.filterState2, f, 0.68f, 0) * 1.1f;
      }

      float sample = rawSample * t.envLevel * t.currentVolume;

      if (t.waveType != 0 && t.waveType != 1) {
        sample *= duckMultiplier;
      }
      if (i >= 4) {
        delaySend += sample * 0.55f;
      }

      t.phase += (2.0f * M_PI * t.currentFreq) / SAMPLE_RATE;
      if (t.phase >= 2.0f * M_PI) t.phase -= 2.0f * M_PI;

      dryL += sample * (1.0f - fmaxf(0.0f, t.pan));
      dryR += sample * (1.0f + fminf(0.0f, t.pan));
    }

    int readIdx = (delayWriteIndex - delayOffset) & DELAY_MASK;
    float echoL = (float)delayBufferL[readIdx] / 26000.0f;
    float echoR = (float)delayBufferR[readIdx] / 26000.0f;

    delayFilterL = delayFilterL * 0.55f + echoL * 0.45f;
    delayFilterR = delayFilterR * 0.55f + echoR * 0.45f;

    float delayInL = softLimit(delaySend + (delayFilterR * 0.48f));
    float delayInR = softLimit(            (delayFilterL * 0.48f));

    delayBufferL[delayWriteIndex] = (int16_t)(delayInL * 26000.0f);
    delayBufferR[delayWriteIndex] = (int16_t)(delayInR * 26000.0f);
    delayWriteIndex = (delayWriteIndex + 1) & DELAY_MASK;

    float totalL = dryL * 0.85f + (echoL * delayMix);
    float totalR = dryR * 0.85f + (echoR * delayMix);

    // Master DC-blocking high-pass (~20 Hz) for maximum headroom & clarity
    float cleanL = totalL - dcPrevInL + 0.997f * dcPrevOutL;
    dcPrevInL = totalL;
    dcPrevOutL = cleanL;

    float cleanR = totalR - dcPrevInR + 0.997f * dcPrevOutR;
    dcPrevInR = totalR;
    dcPrevOutR = cleanR;

    float finalL = softLimit(cleanL);
    float finalR = softLimit(cleanR);

    audioBuffer[s * 2]     = (int32_t)(finalL * 1600000000.0f);
    audioBuffer[s * 2 + 1] = (int32_t)(finalR * 1600000000.0f);
  }

  size_t written;
  i2s_write(I2S_NUM_0, audioBuffer, sizeof(audioBuffer), &written, portMAX_DELAY);
}

// ==========================================
// 4. HARDWARE INPUT CONTROLS
// ==========================================
void readEncoders() {
  int navCLK = digitalRead(ENC_NAV_CLK);
  if (navCLK != lastNavCLK && navCLK == LOW) {
    if (millis() - lastNavTime > 5) {
      resetScreensaverTimer();
      int dir = (digitalRead(ENC_NAV_DT) == LOW) ? -1 : 1;
      if (fnMode) {
        int p = (int)stepProb[currentTrack][currentStep] + dir;
        stepProb[currentTrack][currentStep] = (uint8_t)constrain(p, 1, 4);
      } else {
        currentStep = (currentStep + dir + NUM_STEPS) % NUM_STEPS;
      }
      lastNavTime = millis();
    }
  }
  lastNavCLK = navCLK;

  int pitchCLK = digitalRead(ENC_PITCH_CLK);
  if (pitchCLK != lastPitchCLK && pitchCLK == LOW) {
    resetScreensaverTimer();
    int dir = (digitalRead(ENC_PITCH_DT) == LOW) ? -1 : 1;
    if (fnMode) {
      int n = (int)stepNote[currentTrack][currentStep] + dir;
      stepNote[currentTrack][currentStep] = (int8_t)constrain(n, -12, 12);
    } else {
      semitoneOffset = constrain(semitoneOffset + dir, -12, 12);
      updateTrackFrequencies();
    }
  }
  lastPitchCLK = pitchCLK;

  int decayCLK = digitalRead(ENC_DECAY_CLK);
  if (decayCLK != lastDecayCLK && decayCLK == LOW) {
    resetScreensaverTimer();
    int dir = (digitalRead(ENC_DECAY_DT) == LOW) ? -1 : 1;
    if (fnMode) {
      delayMix = constrain(delayMix + dir * 0.05f, 0.0f, 1.00f);
    } else {
      globalDecayMs = constrain(globalDecayMs + dir * 15, 50, 600);
      updateTrackDecayTimes();
    }
  }
  lastDecayCLK = decayCLK;

  int tempoCLK = digitalRead(ENC_TEMPO_CLK);
  if (tempoCLK != lastTempoCLK && tempoCLK == LOW) {
    resetScreensaverTimer();
    int dir = (digitalRead(ENC_TEMPO_DT) == LOW) ? 1 : -1;
    if (fnMode) {
      swingPercent = constrain(swingPercent + dir * 0.02f, 0.50f, 0.68f);
    } else {
      tempo = constrain(tempo + dir * 2, 60, 200);
      delayOffset = constrain((int)((60000.0f / tempo) * 0.75f * (SAMPLE_RATE / 1000.0f)), 4000, DELAY_BUF_SIZE - 1);
    }
  }
  lastTempoCLK = tempoCLK;
}

void readButtons() {
  unsigned long now = millis();

  int toggleBtn = digitalRead(BTN_TOGGLE);
  if (toggleBtn == LOW && lastToggleBtn == HIGH) {
    toggleBtnDownTime = now;
    toggleLongHandled = false;
    resetScreensaverTimer();
  } else if (toggleBtn == LOW && !toggleLongHandled && (now - toggleBtnDownTime >= 400)) {
    isPlaying = !isPlaying;
    if (isPlaying) lastStepTime = now;
    toggleLongHandled = true;
    resetScreensaverTimer();
  } else if (toggleBtn == HIGH && lastToggleBtn == LOW) {
    if (!toggleLongHandled && (now - toggleBtnDownTime >= 30)) {
      resetScreensaverTimer();
      if (fnMode) {
        trackMuted[currentTrack] = !trackMuted[currentTrack];
      } else {
        sequence[currentTrack][currentStep] = !sequence[currentTrack][currentStep];
      }
    }
  }
  lastToggleBtn = toggleBtn;

  int trackBtn = digitalRead(BTN_TRACK);
  if (trackBtn == LOW && lastTrackBtn == HIGH) {
    trackBtnDownTime = now;
    trackLongHandled = false;
    resetScreensaverTimer();
  } else if (trackBtn == LOW && !trackLongHandled && (now - trackBtnDownTime >= 400)) {
    fnMode = !fnMode;
    trackLongHandled = true;
    resetScreensaverTimer();
  } else if (trackBtn == HIGH && lastTrackBtn == LOW) {
    if (!trackLongHandled && (now - trackBtnDownTime >= 30)) {
      resetScreensaverTimer();
      if (fnMode) {
        fnMode = false; // Quick tap exits FN mode
      } else {
        currentTrack = (currentTrack + 1) % NUM_TRACKS;
      }
    }
  }
  lastTrackBtn = trackBtn;
}

// ==========================================
// 5. SEQUENCER PLAYBACK ENGINE
// ==========================================
void updateSequencer() {
  unsigned long now = millis();
  if (!isPlaying) {
    if (ledOnTime > 0) {
      digitalWrite(LED_BEAT, LOW);
      ledOnTime = 0;
    }
    return;
  }

  if (now - lastStepTime >= currentStepDuration) {
    playbackStep = (playbackStep + 1) % NUM_STEPS;
    lastStepTime = now;

    unsigned long baseInterval = (60000 / tempo) / 4;
    if (playbackStep % 2 == 0) {
      currentStepDuration = (unsigned long)(baseInterval * 2.0f * swingPercent);
    } else {
      currentStepDuration = (unsigned long)(baseInterval * 2.0f * (1.0f - swingPercent));
    }

    bool hasActiveNote = false;
    for (int t = 0; t < NUM_TRACKS; t++) {
      if (!trackMuted[t] && sequence[t][playbackStep]) {
        uint8_t prob = stepProb[t][playbackStep];
        if (prob >= 4 || (esp_random() % 100) < (prob * 25)) {
          triggerTrack(t, playbackStep);
          hasActiveNote = true;
        }
      }
    }

    if (hasActiveNote) {
      digitalWrite(LED_BEAT, HIGH);
      ledOnTime = now;
    }
  }

  if (ledOnTime > 0 && millis() - ledOnTime > 25) {
    digitalWrite(LED_BEAT, LOW);
    ledOnTime = 0;
  }
}

// ==========================================
// 6. MODERN MINIMALIST GRAPHIC UI
// ==========================================
int getStepX(int s) {
  int group = s / 4;
  int posInGroup = s % 4;
  return 5 + (group * 31) + (posInGroup * 7);
}

void drawSequencerUI() {
  display.clearDisplay();

  // Top Bar
  display.setCursor(0, 0);
  display.print(trackNames[currentTrack]);

  if (fnMode) {
    display.fillRect(67, 0, 14, 8, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setCursor(69, 0);
    display.print("FN");
    display.setTextColor(SH110X_WHITE);
    if (trackMuted[currentTrack]) {
      display.fillRect(82, 0, 8, 8, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
      display.setCursor(83, 0);
      display.print("M");
      display.setTextColor(SH110X_WHITE);
    }
  } else if (trackMuted[currentTrack]) {
    display.fillRect(71, 0, 9, 8, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setCursor(73, 0);
    display.print("M");
    display.setTextColor(SH110X_WHITE);
  }

  String rightStr;
  if (fnMode) {
    rightStr = "SW:" + String((int)roundf(swingPercent * 100.0f)) + "%";
  } else {
    rightStr = isPlaying ? (String(tempo) + " BPM") : ("|| " + String(tempo));
  }
  int rightW = rightStr.length() * 6;
  display.setCursor(128 - rightW, 0);
  display.print(rightStr);

  display.drawFastHLine(0, 9, 128, SH110X_WHITE);

  // 16 Step Graphic Pads with Probability Height & "Ghost" Reference Steps
  for (int s = 0; s < NUM_STEPS; s++) {
    int x = getStepX(s);
    int y = 14;

    bool thisTrackNote = sequence[currentTrack][s];
    bool otherTrackNote = false;
    for (int t = 0; t < NUM_TRACKS; t++) {
      if (t != currentTrack && sequence[t][s]) {
        otherTrackNote = true;
        break;
      }
    }

    if (thisTrackNote) {
      uint8_t prob = stepProb[currentTrack][s];
      if (prob >= 4) {
        display.fillRoundRect(x, y, 5, 6, 1, SH110X_WHITE);
      } else {
        int h = prob + 1; // 75%=4px, 50%=3px, 25%=2px
        display.fillRect(x, y + (6 - h), 5, h, SH110X_WHITE);
      }
    } else if (otherTrackNote) {
      display.fillRect(x, y + 2, 5, 2, SH110X_WHITE);
    } else {
      display.drawPixel(x + 2, y + 2, SH110X_WHITE);
    }

    if (s == currentStep) {
      display.drawRect(x - 1, y - 1, 7, 8, SH110X_WHITE);
    }

    if (s == playbackStep) {
      display.fillRect(x + 1, y + 8, 3, 2, SH110X_WHITE);
    }
  }

  // Waveform Visualizer
  Track &ct = tracks[currentTrack];
  int waveY = 32, waveHeight = 8;
  display.drawFastHLine(0, waveY, 128, SH110X_WHITE);
  int numSamples = 64;
  for (int i = 0; i < numSamples - 1; i++) {
    float p1 = (i / (float)numSamples) * 4.0f * M_PI;
    float p2 = ((i + 1) / (float)numSamples) * 4.0f * M_PI;
    float s1 = 0, s2 = 0;

    if (ct.waveType == 0) {
      float env1 = 1.0f - (i / (float)numSamples);
      float env2 = 1.0f - ((i + 1) / (float)numSamples);
      s1 = sinf(p1 * env1) * env1;
      s2 = sinf(p2 * env2) * env2;
    } else if (ct.waveType == 1) {
      s1 = ((i % 4 == 0) ? 0.9f : -0.7f) * (1.0f - (i / (float)numSamples));
      s2 = (((i + 1) % 4 == 0) ? 0.9f : -0.7f) * (1.0f - ((i + 1) / (float)numSamples));
    } else if (ct.waveType == 2) {
      s1 = sinf(p1) + 0.38f * sinf(p1 * 2.0f);
      s2 = sinf(p2) + 0.38f * sinf(p2 * 2.0f);
    } else if (ct.waveType == 3) {
      s1 = (2.0f * (fmodf(p1, 2.0f * M_PI) / (2.0f * M_PI)) - 1.0f) * 0.8f;
      s2 = (2.0f * (fmodf(p2, 2.0f * M_PI) / (2.0f * M_PI)) - 1.0f) * 0.8f;
    } else if (ct.waveType == 4) {
      s1 = sinf(p1) + 0.35f * sinf(p1 * 2.0f);
      s2 = sinf(p2) + 0.35f * sinf(p2 * 2.0f);
    } else if (ct.waveType == 5) {
      s1 = 2.0f * fabsf(2.0f * (fmodf(p1, 2.0f * M_PI) / (2.0f * M_PI)) - 1.0f) - 1.0f;
      s2 = 2.0f * fabsf(2.0f * (fmodf(p2, 2.0f * M_PI) / (2.0f * M_PI)) - 1.0f) - 1.0f;
    }

    int y1 = constrain(waveY - (int)(s1 * (waveHeight / 2.0f)), waveY - waveHeight / 2, waveY + waveHeight / 2);
    int y2 = constrain(waveY - (int)(s2 * (waveHeight / 2.0f)), waveY - waveHeight / 2, waveY + waveHeight / 2);
    display.drawLine(i * 2, y1, (i + 1) * 2, y2, SH110X_WHITE);
  }

  // Minimalist Track Circles
  int circleY = 46;
  for (int t = 0; t < NUM_TRACKS; t++) {
    int cx = 10 + (t * 15);
    bool hasNotes = false;
    for (int s = 0; s < NUM_STEPS; s++) {
      if (sequence[t][s]) hasNotes = true;
    }

    if (t == currentTrack) {
      display.drawCircle(cx, circleY, 5, SH110X_WHITE);
      if (trackMuted[t]) {
        display.drawLine(cx - 2, circleY - 2, cx + 2, circleY + 2, SH110X_WHITE);
        display.drawLine(cx - 2, circleY + 2, cx + 2, circleY - 2, SH110X_WHITE);
      } else {
        display.fillCircle(cx, circleY, 3, SH110X_WHITE);
      }
    } else if (trackMuted[t]) {
      display.drawLine(cx - 2, circleY - 2, cx + 2, circleY + 2, SH110X_WHITE);
      display.drawLine(cx - 2, circleY + 2, cx + 2, circleY - 2, SH110X_WHITE);
    } else if (tracks[t].active && tracks[t].envLevel > 0.08f) {
      display.fillCircle(cx, circleY, 4, SH110X_WHITE);
    } else if (hasNotes) {
      display.drawCircle(cx, circleY, 3, SH110X_WHITE);
      display.drawPixel(cx, circleY, SH110X_WHITE);
    } else {
      display.drawCircle(cx, circleY, 2, SH110X_WHITE);
    }
  }

  // Unified Bottom Line: Key/Offset, Step Note, Step Prob, Decay (or Delay in FN)
  int noteIndex = (semitoneOffset % 12 + 12) % 12;
  display.setCursor(0, 56);
  display.print(keyNames[noteIndex]);
  if (semitoneOffset > 0) { display.print("+"); display.print(semitoneOffset); }
  else if (semitoneOffset < 0) { display.print(semitoneOffset); }

  int8_t sn = stepNote[currentTrack][currentStep];
  display.setCursor(34, 56);
  display.print("N");
  if (sn > 0) { display.print("+"); display.print(sn); }
  else { display.print(sn); }

  display.setCursor(64, 56);
  display.print(stepProb[currentTrack][currentStep] * 25);
  display.print("%");

  display.setCursor(92, 56);
  if (fnMode) {
    display.print("W:");
    display.print((int)roundf(delayMix * 100.0f));
    display.print("%");
  } else {
    display.print("D:");
    display.print(globalDecayMs);
  }

  display.display();
}

// ==========================================
// 7. SLEEK METRONOME SCREENSAVER (BPM-SYNCED, NO TEXT)
// ==========================================
void drawScreensaver() {
  display.clearDisplay();

  const int cx = 64;
  const int topY = 6;
  const int baseY = 58;
  const int halfBase = 24;
  const int plinthY = 51;

  // 1. Outer Metronome Triangle Body & Base Plinth
  display.drawLine(cx, topY, cx - halfBase, baseY, SH110X_WHITE);
  display.drawLine(cx, topY, cx + halfBase, baseY, SH110X_WHITE);
  display.drawFastHLine(cx - halfBase, baseY, halfBase * 2 + 1, SH110X_WHITE);
  display.drawFastHLine(cx - 21, plinthY, 43, SH110X_WHITE);

  // Subtle center scale line & end-stop ticks
  display.drawFastVLine(cx, topY + 6, 4, SH110X_WHITE);
  display.drawPixel(cx - 12, 18, SH110X_WHITE);
  display.drawPixel(cx + 12, 18, SH110X_WHITE);

  // 2. BPM-Synced Pendulum Stick (1 swing per beat)
  float beatPhase = ((float)millis() * (float)tempo) / 120000.0f;
  float angle = sinf(beatPhase * 2.0f * M_PI) * 0.40f; // ~ +/- 23 deg

  int pivotX = cx;
  int pivotY = plinthY;
  int stickLen = 38;
  int tipX = pivotX + (int)(sinf(angle) * stickLen);
  int tipY = pivotY - (int)(cosf(angle) * stickLen);

  display.drawLine(pivotX, pivotY, tipX, tipY, SH110X_WHITE);
  display.fillCircle(pivotX, pivotY, 2, SH110X_WHITE);

  // 3. Weighted Bob on the Stick
  int bobR = 24;
  int bobX = pivotX + (int)(sinf(angle) * bobR);
  int bobY = pivotY - (int)(cosf(angle) * bobR);
  display.fillTriangle(
    bobX, bobY - 3,
    bobX - 3, bobY + 3,
    bobX + 3, bobY + 3,
    SH110X_WHITE
  );

  display.display();
}

// ==========================================
// 8. DISPLAY UPDATE ROUTINE
// ==========================================
void updateDisplay() {
  static unsigned long lastUpdate = 0;
  bool isIdle = (millis() - lastUserInteraction > 30000);
  unsigned long refreshInterval = isIdle ? 45 : 100;

  if (millis() - lastUpdate > refreshInterval) {
    if (isIdle) {
      drawScreensaver();
    } else {
      drawSequencerUI();
    }
    lastUpdate = millis();
  }
}

// ==========================================
// 9. MAIN ARDUINO LOOP
// ==========================================
void loop() {
  readEncoders();
  readButtons();
  updateSequencer();
  generateAudio();
  updateDisplay();
}
