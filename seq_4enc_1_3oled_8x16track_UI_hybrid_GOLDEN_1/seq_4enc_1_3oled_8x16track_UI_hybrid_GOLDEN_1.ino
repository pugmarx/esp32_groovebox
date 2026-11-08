#include <driver/i2s.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// I2S Audio
#define I2S_BCK 26
#define I2S_WS 25
#define I2S_DOUT 27
#define SAMPLE_RATE 44100
#define BUFFER_SIZE 64

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_DC 16
#define OLED_CS 4
#define OLED_RESET 17
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS);

// Encoders
#define ENC_NAV_CLK 34
#define ENC_NAV_DT 35
#define ENC_PITCH_CLK 5
#define ENC_PITCH_DT 13
#define ENC_FILTER_CLK 33
#define ENC_FILTER_DT 14
#define ENC_TEMPO_CLK 21
#define ENC_TEMPO_DT 19

// Buttons & LED
#define BTN_TOGGLE 22
#define BTN_TRACK 32
#define LED_BEAT 2

// Sequencer parameters
#define NUM_TRACKS 8
#define NUM_STEPS 16

struct Track {
  bool active;
  float phase;
  float frequency;
  int waveType; 
  float pan;
  float volume;
  float envLevel;
  int envStage; 
  int envCounter;
  int attackTime;
  int decayTime;
  int releaseTime;
  float sustainLevel;
  float filterState;
  float filterCutoff; 
};

Track tracks[NUM_TRACKS];
bool sequence[NUM_TRACKS][NUM_STEPS] = {0};

int currentTrack = 0;
int currentStep = 0;
int playbackStep = 0;
int tempo = 120;
int globalPitch = 440;
int globalFilter = 5000;

unsigned long lastStepTime = 0;
unsigned long stepInterval = 0;
unsigned long ledOnTime = 0;
unsigned long lastNavTime = 0;

int lastNavCLK = HIGH, lastPitchCLK = HIGH, lastFilterCLK = HIGH, lastTempoCLK = HIGH;
int lastToggleBtn = HIGH, lastTrackBtn = HIGH;
const char* waveNames[] = {"SIN", "TRI"};

void initializeTracks() {
  tracks[0] = {false, 0, 220, 0, -0.2, 0.95, 0, 0, 0, 30, 600, 1200, 0.0, 0, 0.8};
  tracks[1] = {false, 0, 330, 0, 0.2, 0.9, 0, 0, 0, 40, 700, 1300, 0.0, 0, 0.8};
  tracks[2] = {false, 0, 440, 1, -0.3, 0.7, 0, 0, 0, 200, 1000, 1800, 0.3, 0, 0.7};
  tracks[3] = {false, 0, 550, 1, 0.3, 0.65, 0, 0, 0, 200, 1100, 1900, 0.3, 0, 0.7};
  tracks[4] = {false, 0, 660, 0, -0.2, 0.6, 0, 0, 0, 150, 900, 1500, 0.4, 0, 0.6};
  tracks[5] = {false, 0, 880, 1, 0.2, 0.55, 0, 0, 0, 150, 800, 1400, 0.4, 0, 0.6};
  tracks[6] = {false, 0, 1100, 0, -0.4, 0.5, 0, 0, 0, 100, 700, 1200, 0.5, 0, 0.5};
  tracks[7] = {false, 0, 1320, 1, 0.4, 0.45, 0, 0, 0, 100, 600, 1000, 0.5, 0, 0.5};
}

void setup() {
  Serial.begin(115200);
  
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 128,
    .use_apll = false
  };
  
  i2s_pin_config_t pin_config = {.bck_io_num = 26, .ws_io_num = 25, .data_out_num = 27, .data_in_num = I2S_PIN_NO_CHANGE};
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);
  
  initializeTracks();
  
  SPI.begin();
  display.begin(0x3C, true);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.println("Groovebox v3.6");
  display.println("Single Core Stable");
  display.display();
  delay(1000);
  
  pinMode(ENC_NAV_CLK, INPUT_PULLUP); pinMode(ENC_NAV_DT, INPUT_PULLUP);
  pinMode(ENC_PITCH_CLK, INPUT_PULLUP); pinMode(ENC_PITCH_DT, INPUT_PULLUP);
  pinMode(ENC_FILTER_CLK, INPUT_PULLUP); pinMode(ENC_FILTER_DT, INPUT_PULLUP);
  pinMode(ENC_TEMPO_CLK, INPUT_PULLUP); pinMode(ENC_TEMPO_DT, INPUT_PULLUP);
  pinMode(BTN_TOGGLE, INPUT_PULLUP); pinMode(BTN_TRACK, INPUT_PULLUP);
  pinMode(LED_BEAT, OUTPUT);
  
  stepInterval = (60000 / tempo) / 4;
  Serial.println("Groovebox initialized - Single core mode");
}

void triggerTrack(int trackNum) {
  Track &t = tracks[trackNum];
  t.active = true;
  t.phase = 0;
  t.envStage = 1;
  t.envCounter = 0;
  t.envLevel = 0;
  t.filterState = 0;
}

void generateAudio() {
  int16_t audioBuffer[BUFFER_SIZE * 2];
  
  for (int s = 0; s < BUFFER_SIZE; s++) {
    float mixL = 0, mixR = 0;
    
    for (int i = 0; i < NUM_TRACKS; i++) {
      Track &t = tracks[i];
      if (!t.active) continue;

      float sample = (t.waveType == 0) ? sin(t.phase) : (2.0f / PI) * asin(sin(t.phase));
      
      if (t.envStage == 1) {
        t.envCounter++;
        t.envLevel = (float)t.envCounter / t.attackTime;
        if (t.envCounter >= t.attackTime) {
          t.envStage = 2;
          t.envCounter = 0;
        }
      }
      else if (t.envStage == 2) {
        t.envCounter++;
        float dec = (float)t.envCounter / t.decayTime;
        t.envLevel = 1.0f - (dec * (1.0f - t.sustainLevel));
        if (t.envCounter >= t.decayTime) {
          t.envStage = 3;
          t.envLevel = t.sustainLevel;
        }
      }
      else if (t.envStage == 4) {
        t.envCounter++;
        t.envLevel = t.sustainLevel * (1.0f - ((float)t.envCounter / t.releaseTime));
        if (t.envCounter >= t.releaseTime || t.envLevel <= 0.001f) {
          t.active = false;
          t.envStage = 0;
          t.envLevel = 0;
        }
      }

      sample *= t.envLevel * t.volume;
      t.filterState = t.filterState * (1.0f - t.filterCutoff) + sample * t.filterCutoff;
      sample = t.filterState;

      t.phase += (2.0f * PI * t.frequency) / SAMPLE_RATE;
      if (t.phase >= 2.0f * PI) t.phase -= 2.0f * PI;

      mixL += sample * (1.0f - max(0.0f, t.pan));
      mixR += sample * (1.0f + min(0.0f, t.pan));
    }
    
    audioBuffer[s * 2] = (int16_t)constrain(mixL * 1500, -32000, 32000);
    audioBuffer[s * 2 + 1] = (int16_t)constrain(mixR * 1500, -32000, 32000);
  }
  
  size_t written;
  i2s_write(I2S_NUM_0, audioBuffer, sizeof(audioBuffer), &written, portMAX_DELAY);
}

void readEncoders() {
  int navCLK = digitalRead(ENC_NAV_CLK);
  if (navCLK != lastNavCLK && navCLK == LOW) {
    if (millis() - lastNavTime > 5) {
      if (digitalRead(ENC_NAV_DT) == LOW) currentStep = (currentStep - 1 + NUM_STEPS) % NUM_STEPS;
      else currentStep = (currentStep + 1) % NUM_STEPS;
      lastNavTime = millis();
    }
  }
  lastNavCLK = navCLK;

  int pitchCLK = digitalRead(ENC_PITCH_CLK);
  if (pitchCLK != lastPitchCLK && pitchCLK == LOW) {
    globalPitch = (digitalRead(ENC_PITCH_DT) == LOW) ? globalPitch - 20 : globalPitch + 20;
    globalPitch = constrain(globalPitch, 200, 2000);
    float ratio = globalPitch / 440.0;
    tracks[0].frequency = 220 * ratio; tracks[1].frequency = 330 * ratio; tracks[2].frequency = 440 * ratio;
    tracks[3].frequency = 550 * ratio; tracks[4].frequency = 660 * ratio; tracks[5].frequency = 880 * ratio;
    tracks[6].frequency = 1100 * ratio; tracks[7].frequency = 1320 * ratio;
  }
  lastPitchCLK = pitchCLK;

  int filterCLK = digitalRead(ENC_FILTER_CLK);
  if (filterCLK != lastFilterCLK && filterCLK == LOW) {
    globalFilter = (digitalRead(ENC_FILTER_DT) == LOW) ? globalFilter - 200 : globalFilter + 200;
    globalFilter = constrain(globalFilter, 200, 8000);
    for (int i = 0; i < NUM_TRACKS; i++) {
      tracks[i].filterCutoff = (float)globalFilter / 10000.0;
      tracks[i].filterCutoff = constrain(tracks[i].filterCutoff, 0.1, 0.95);
    }
  }
  lastFilterCLK = filterCLK;

  int tempoCLK = digitalRead(ENC_TEMPO_CLK);
  if (tempoCLK != lastTempoCLK && tempoCLK == LOW) {
    tempo = (digitalRead(ENC_TEMPO_DT) == LOW) ? tempo + 2 : tempo - 2;
    tempo = constrain(tempo, 80, 180);
    stepInterval = (60000 / tempo) / 4;
  }
  lastTempoCLK = tempoCLK;
}

void readButtons() {
  int toggleBtn = digitalRead(BTN_TOGGLE);
  if (toggleBtn == LOW && lastToggleBtn == HIGH) {
    sequence[currentTrack][currentStep] = !sequence[currentTrack][currentStep];
    delay(50);
  }
  lastToggleBtn = toggleBtn;
  
  int trackBtn = digitalRead(BTN_TRACK);
  if (trackBtn == LOW && lastTrackBtn == HIGH) {
    currentTrack = (currentTrack + 1) % NUM_TRACKS;
    delay(50);
  }
  lastTrackBtn = trackBtn;
}

void updateSequencer() {
  if (millis() - lastStepTime >= stepInterval) {
    playbackStep = (playbackStep + 1) % NUM_STEPS;
    lastStepTime = millis();
    
    bool hasActiveNote = false;
    for (int t = 0; t < NUM_TRACKS; t++) {
      if (sequence[t][playbackStep]) {
        triggerTrack(t);
        hasActiveNote = true;
      } else {
        if (tracks[t].active && tracks[t].envStage == 3) {
          tracks[t].envStage = 4;
          tracks[t].envCounter = 0;
        }
      }
    }
    
    if (hasActiveNote) {
      digitalWrite(LED_BEAT, HIGH);
      ledOnTime = millis();
    }
  }
  
  if (ledOnTime > 0 && millis() - ledOnTime > 20) {
    digitalWrite(LED_BEAT, LOW);
    ledOnTime = 0;
  }
}

void drawWaveform(int trackNum) {
  Track &t = tracks[trackNum];
  int waveY = 32, waveHeight = 7, waveWidth = 128, waveX = 0;
  display.drawFastHLine(waveX, waveY, waveWidth, SH110X_WHITE);
  int numSamples = 64; float cycles = 2.0;
  for (int i = 0; i < numSamples - 1; i++) {
    float p1 = (i / (float)numSamples) * cycles * 2.0 * PI;
    float p2 = ((i + 1) / (float)numSamples) * cycles * 2.0 * PI;
    float s1 = (t.waveType == 0) ? sin(p1) : (2.0 / PI) * asin(sin(p1));
    float s2 = (t.waveType == 0) ? sin(p2) : (2.0 / PI) * asin(sin(p2));
    int y1 = constrain(waveY - (int)(s1 * (waveHeight / 2.0)), waveY - waveHeight/2, waveY + waveHeight/2);
    int y2 = constrain(waveY - (int)(s2 * (waveHeight / 2.0)), waveY - waveHeight/2, waveY + waveHeight/2);
    display.drawLine(waveX + (i * waveWidth / numSamples), y1, waveX + ((i + 1) * waveWidth / numSamples), y2, SH110X_WHITE);
  }
}

void updateDisplay() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 200) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("T"); display.print(currentTrack + 1);
    display.print(" "); display.print(waveNames[tracks[currentTrack].waveType]);
    display.setCursor(40, 0); display.print("S"); display.print(currentStep + 1);
    
    // Right-aligned BPM
    String bpmText = String(tempo) + " BPM";
    int bpmWidth = bpmText.length() * 6;
    display.setCursor(128 - bpmWidth, 0); 
    display.print(bpmText);
    
    for (int t = 0; t < NUM_TRACKS; t++) {
      if (tracks[t].active && tracks[t].envLevel > 0.1) display.fillCircle(110 + (t * 2), 3, 1, SH110X_WHITE);
    }
    
    display.drawFastHLine(0, 9, 128, SH110X_WHITE);
    display.setCursor(0, 12);
    for (int s = 0; s < NUM_STEPS; s++) {
      if (s == currentStep) display.print("[");
      if (sequence[currentTrack][s]) display.print("X");
      else {
        bool o = false; for(int t=0; t<NUM_TRACKS; t++) if(t!=currentTrack && sequence[t][s]) o = true;
        display.print(o ? "-" : ".");
      }
      if (s == currentStep) display.print("]");
      if (s == 7) display.print(" ");
    }
    display.setCursor(0, 20);
    for (int s = 0; s < NUM_STEPS; s++) { 
      display.print(s == playbackStep ? ">" : " "); 
      if (s == 7) display.print(" "); 
    }
    
    // Full-width waveform
    drawWaveform(currentTrack);
    for (int t = 0; t < NUM_TRACKS; t++) {
      bool n = false; for (int s = 0; s < NUM_STEPS; s++) if (sequence[t][s]) n = true;
      int xP = t * 16;
      if (n) { display.fillRect(xP, 42, 12, 8, SH110X_WHITE); display.setTextColor(SH110X_BLACK); }
      else { display.drawRect(xP, 42, 12, 8, SH110X_WHITE); display.setTextColor(SH110X_WHITE); }
      display.setCursor(xP + 3, 43); display.print(t + 1); display.setTextColor(SH110X_WHITE);
    }
    display.setCursor(0, 56); display.print("P:");
    display.print(globalPitch);
    display.setCursor(64, 56); display.print("F:");
    display.print(globalFilter);
    display.display();
    lastUpdate = millis();
  }
}

void loop() {
  readEncoders();
  readButtons();
  updateSequencer();
  generateAudio();
  updateDisplay();
}