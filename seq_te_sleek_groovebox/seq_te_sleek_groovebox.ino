#include <driver/i2s.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <math.h>

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

int currentTrack = 0;
int currentStep = 0;
int playbackStep = 0;
int tempo = 120;
int semitoneOffset = 0;
int globalDecayMs = 160;

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
float reelAngle = 0.0f;
int animFrame = 0;

int lastNavCLK = HIGH, lastPitchCLK = HIGH, lastDecayCLK = HIGH, lastTempoCLK = HIGH;
int lastToggleBtn = HIGH, lastTrackBtn = HIGH;
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
  tracks[5].baseDecaySamples = (int)(baseDecaySamples * 1.1f);
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

  // Large 16 x 256 DMA Buffer (~93ms) prevents audio dropouts
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 16,
    .dma_buf_len = 256,
    .use_apll = false
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
  memset(delayBufferL, 0, sizeof(delayBufferL));
  memset(delayBufferR, 0, sizeof(delayBufferR));

  SPI.begin();
  display.begin(0x3C, true);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(22, 18);
  display.println("GROOVEBOX v4");
  display.setCursor(18, 36);
  display.println("Smooth Clean Audio");
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

void triggerTrack(int trackNum) {
  Track &t = tracks[trackNum];
  t.active = true;
  t.phase = 0.0f;
  t.envStage = 1;
  t.envCounter = 0;
  t.envLevel = 0.0f;
  t.filterState1 = 0.0f;
  t.filterState2 = 0.0f;

  // 1. Human Micro-Dynamics (Velocity Drift: subtle ±7% natural acoustic variation)
  float humanVelocity = 0.93f + ((float)(esp_random() % 100) / 100.0f) * 0.14f;
  t.currentVolume = t.baseVolume * humanVelocity;

  // 2. Micro-Decay Drift: subtle ±8% resonance variance per note
  float decayDrift = 0.92f + ((float)(esp_random() % 100) / 100.0f) * 0.16f;
  t.currentDecaySamples = (int)(t.baseDecaySamples * decayDrift);

  // 3. Funk Bass Octave Pop: 12% chance for bass (T3, T4) or pluck (T5) to pop an octave up
  float pitchMult = 1.0f;
  if ((trackNum == 2 || trackNum == 3 || trackNum == 4) && (esp_random() % 100 < 12)) {
    pitchMult = 2.0f; // 1 octave jump
  }

  if (t.waveType == 0) {
    t.currentFreq = 145.0f;
    sidechainAmount = 0.35f;
  } else {
    t.currentFreq = t.baseFreq * pitchMult;
  }
}

inline float softLimit(float x) {
  return x / sqrtf(1.0f + x * x);
}

// ==========================================
// 3. AUDIO ENGINE
// ==========================================
void generateAudio() {
  int16_t audioBuffer[BUFFER_SIZE * 2];

  for (int s = 0; s < BUFFER_SIZE; s++) {
    sidechainAmount *= 0.99965f;
    float duckMultiplier = 1.0f - sidechainAmount;

    float dryL = 0.0f, dryR = 0.0f;
    float delaySend = 0.0f;

    for (int i = 0; i < NUM_TRACKS; i++) {
      Track &t = tracks[i];
      if (!t.active) continue;

      float rawSample = 0.0f;

      if (t.waveType == 0) {
        rawSample = sinf(t.phase);
        t.currentFreq = t.baseFreq + (t.currentFreq - t.baseFreq) * 0.9991f;
      }
      else if (t.waveType == 1) {
        noiseSeed = noiseSeed * 1664525 + 1013904223;
        float noise = ((float)(noiseSeed & 0xFFFF) / 32768.0f) - 1.0f;
        float hp = noise - t.filterState1;
        t.filterState1 = t.filterState1 * 0.30f + noise * 0.70f;
        t.filterState2 = t.filterState2 * 0.55f + hp * 0.45f;
        rawSample = t.filterState2 * 0.85f;
      }
      else if (t.waveType == 2) {
        float fundamental = sinf(t.phase);
        float overtone1   = sinf(t.phase * 2.0f) * 0.38f;
        float overtone2   = sinf(t.phase * 3.0f) * 0.16f;
        rawSample = fundamental + overtone1 + overtone2;
      }
      else if (t.waveType == 3) {
        float saw = 2.0f * (t.phase / (2.0f * M_PI)) - 1.0f;
        t.filterState1 = t.filterState1 * 0.84f + saw * 0.16f;
        t.filterState2 = t.filterState2 * 0.84f + t.filterState1 * 0.16f;
        rawSample = t.filterState2 * 1.5f;
      }
      else if (t.waveType == 4) {
        float fundamental = sinf(t.phase);
        float overtone1   = sinf(t.phase * 2.0f) * 0.35f;
        float overtone2   = sinf(t.phase * 3.0f) * 0.15f;
        rawSample = fundamental + overtone1 + overtone2;
      }
      else if (t.waveType == 5) {
        float saw = 2.0f * (t.phase / (2.0f * M_PI)) - 1.0f;
        float tri = 2.0f * fabsf(saw) - 1.0f;
        rawSample = tri * 0.7f + sinf(t.phase) * 0.3f;
      }

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

      float sample = rawSample * t.envLevel * t.currentVolume;

      if (t.waveType != 0 && t.waveType != 1) {
        sample *= duckMultiplier;
      }
      if (i >= 4) {
        delaySend += sample * 0.35f;
      }

      t.phase += (2.0f * M_PI * t.currentFreq) / SAMPLE_RATE;
      if (t.phase >= 2.0f * M_PI) t.phase -= 2.0f * M_PI;

      dryL += sample * (1.0f - fmaxf(0.0f, t.pan));
      dryR += sample * (1.0f + fminf(0.0f, t.pan));
    }

    int readIdx = (delayWriteIndex - delayOffset) & DELAY_MASK;
    float echoL = (float)delayBufferL[readIdx] / 30000.0f;
    float echoR = (float)delayBufferR[readIdx] / 30000.0f;

    delayFilterL = delayFilterL * 0.70f + echoL * 0.30f;
    delayFilterR = delayFilterR * 0.70f + echoR * 0.30f;

    float delayInL = softLimit(delaySend + (delayFilterR * 0.32f));
    float delayInR = softLimit(            (delayFilterL * 0.32f));

    delayBufferL[delayWriteIndex] = (int16_t)(delayInL * 26000.0f);
    delayBufferR[delayWriteIndex] = (int16_t)(delayInR * 26000.0f);
    delayWriteIndex = (delayWriteIndex + 1) & DELAY_MASK;

    float totalL = dryL * 0.85f + (echoL * 0.28f);
    float totalR = dryR * 0.85f + (echoR * 0.28f);

    float finalL = softLimit(totalL);
    float finalR = softLimit(totalR);

    audioBuffer[s * 2]     = (int16_t)(finalL * 25000.0f);
    audioBuffer[s * 2 + 1] = (int16_t)(finalR * 25000.0f);
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
      if (digitalRead(ENC_NAV_DT) == LOW) currentStep = (currentStep - 1 + NUM_STEPS) % NUM_STEPS;
      else currentStep = (currentStep + 1) % NUM_STEPS;
      lastNavTime = millis();
    }
  }
  lastNavCLK = navCLK;

  int pitchCLK = digitalRead(ENC_PITCH_CLK);
  if (pitchCLK != lastPitchCLK && pitchCLK == LOW) {
    resetScreensaverTimer();
    semitoneOffset = (digitalRead(ENC_PITCH_DT) == LOW) ? semitoneOffset - 1 : semitoneOffset + 1;
    semitoneOffset = constrain(semitoneOffset, -12, 12);
    updateTrackFrequencies();
  }
  lastPitchCLK = pitchCLK;

  int decayCLK = digitalRead(ENC_DECAY_CLK);
  if (decayCLK != lastDecayCLK && decayCLK == LOW) {
    resetScreensaverTimer();
    globalDecayMs = (digitalRead(ENC_DECAY_DT) == LOW) ? globalDecayMs - 15 : globalDecayMs + 15;
    globalDecayMs = constrain(globalDecayMs, 50, 600);
    updateTrackDecayTimes();
  }
  lastDecayCLK = decayCLK;

  int tempoCLK = digitalRead(ENC_TEMPO_CLK);
  if (tempoCLK != lastTempoCLK && tempoCLK == LOW) {
    resetScreensaverTimer();
    tempo = (digitalRead(ENC_TEMPO_DT) == LOW) ? tempo + 2 : tempo - 2;
    tempo = constrain(tempo, 60, 200);
    delayOffset = constrain((int)((60000.0f / tempo) * 0.75f * (SAMPLE_RATE / 1000.0f)), 4000, DELAY_BUF_SIZE - 1);
  }
  lastTempoCLK = tempoCLK;
}

void readButtons() {
  int toggleBtn = digitalRead(BTN_TOGGLE);
  if (toggleBtn == LOW && lastToggleBtn == HIGH) {
    resetScreensaverTimer();
    sequence[currentTrack][currentStep] = !sequence[currentTrack][currentStep];
    delay(40);
  }
  lastToggleBtn = toggleBtn;

  int trackBtn = digitalRead(BTN_TRACK);
  if (trackBtn == LOW && lastTrackBtn == HIGH) {
    resetScreensaverTimer();
    currentTrack = (currentTrack + 1) % NUM_TRACKS;
    delay(40);
  }
  lastTrackBtn = trackBtn;
}

// ==========================================
// 5. SEQUENCER PLAYBACK ENGINE
// ==========================================
void updateSequencer() {
  unsigned long now = millis();
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
      if (sequence[t][playbackStep]) {
        triggerTrack(t);
        hasActiveNote = true;
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

  String bpmStr = String(tempo) + " BPM";
  int bpmW = bpmStr.length() * 6;
  display.setCursor(128 - bpmW, 0);
  display.print(bpmStr);

  display.drawFastHLine(0, 9, 128, SH110X_WHITE);

  // 16 Step Graphic Pads with "Ghost" Reference Steps
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
      display.fillRoundRect(x, y, 5, 6, 1, SH110X_WHITE);
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
      display.fillCircle(cx, circleY, 3, SH110X_WHITE);
    } else if (tracks[t].active && tracks[t].envLevel > 0.08f) {
      display.fillCircle(cx, circleY, 4, SH110X_WHITE);
    } else if (hasNotes) {
      display.drawCircle(cx, circleY, 3, SH110X_WHITE);
      display.drawPixel(cx, circleY, SH110X_WHITE);
    } else {
      display.drawCircle(cx, circleY, 2, SH110X_WHITE);
    }
  }

  // Clean Bottom Line
  int noteIndex = (semitoneOffset % 12 + 12) % 12;
  display.setCursor(0, 56);
  display.print("KEY:");
  display.print(keyNames[noteIndex]);
  if (semitoneOffset > 0) { display.print("+"); display.print(semitoneOffset); }
  else if (semitoneOffset < 0) { display.print(semitoneOffset); }

  display.setCursor(68, 56);
  display.print("DECAY:"); display.print(globalDecayMs); display.print("ms");

  display.display();
}

// ==========================================
// 7. SLEEK VINTAGE CASSETTE SCREENSAVER (NO TEXT)
// ==========================================
void drawScreensaver() {
  display.clearDisplay();

  reelAngle += 0.20f;
  if (reelAngle >= 2.0f * M_PI) reelAngle -= 2.0f * M_PI;
  animFrame++;

  // 1. Minimalist Cassette Body Outline
  int cX = 14, cY = 7, cW = 100, cH = 50;
  display.drawRoundRect(cX, cY, cW, cH, 4, SH110X_WHITE);

  // Top header groove
  display.drawFastHLine(cX + 8, cY + 4, cW - 16, SH110X_WHITE);

  // 2. Center Window Frame
  int wX = cX + 14, wY = cY + 10, wW = 72, wH = 26;
  display.drawRect(wX, wY, wW, wH, SH110X_WHITE);

  // 3. Dual Rotating Tape Hubs / Spools
  int leftHubX = wX + 16, rightHubX = wX + 56, hubY = wY + 13;
  int hubRadius = 9;

  display.drawCircle(leftHubX,  hubY, hubRadius, SH110X_WHITE);
  display.drawCircle(rightHubX, hubY, hubRadius, SH110X_WHITE);
  display.drawCircle(leftHubX,  hubY, 3, SH110X_WHITE);
  display.drawCircle(rightHubX, hubY, 3, SH110X_WHITE);

  for (int i = 0; i < 3; i++) {
    float a = reelAngle + (i * 2.0f * M_PI / 3.0f);
    int x1 = leftHubX + (int)(cosf(a) * 3);
    int y1 = hubY     + (int)(sinf(a) * 3);
    int x2 = leftHubX + (int)(cosf(a) * (hubRadius - 1));
    int y2 = hubY     + (int)(sinf(a) * (hubRadius - 1));
    display.drawLine(x1, y1, x2, y2, SH110X_WHITE);

    int rx1 = rightHubX + (int)(cosf(a) * 3);
    int ry1 = hubY      + (int)(sinf(a) * 3);
    int rx2 = rightHubX + (int)(cosf(a) * (hubRadius - 1));
    int ry2 = hubY      + (int)(sinf(a) * (hubRadius - 1));
    display.drawLine(rx1, ry1, rx2, ry2, SH110X_WHITE);
  }

  // Tape Ribbon
  display.drawFastHLine(leftHubX, hubY - hubRadius, rightHubX - leftHubX, SH110X_WHITE);

  // 4. Subtle Floating Musical Note
  int noteY = (120 - (animFrame * 2)) % 22 + (wY + 4);
  int noteX = 64;
  display.fillCircle(noteX, noteY, 1, SH110X_WHITE);
  display.drawFastVLine(noteX + 1, noteY - 5, 5, SH110X_WHITE);
  display.drawFastHLine(noteX + 1, noteY - 5, 2, SH110X_WHITE);

  // 5. Bottom Tape Head Cutout (Trapezoid)
  int trapTopY = cY + cH - 11;
  int trapBotY = cY + cH - 1;
  display.drawFastHLine(cX + 28, trapTopY, 44, SH110X_WHITE);
  display.drawLine(cX + 28, trapTopY, cX + 22, trapBotY, SH110X_WHITE);
  display.drawLine(cX + 72, trapTopY, cX + 78, trapBotY, SH110X_WHITE);
  display.drawCircle(cX + 32, trapTopY + 5, 2, SH110X_WHITE);
  display.drawCircle(cX + 68, trapTopY + 5, 2, SH110X_WHITE);

  display.display();
}

// ==========================================
// 8. DISPLAY UPDATE ROUTINE
// ==========================================
void updateDisplay() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 100) {
    if (millis() - lastUserInteraction > 30000) {
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
