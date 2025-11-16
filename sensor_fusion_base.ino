// ===================================================
// INCLUDE GLOBALI
// ===================================================
#include <Adafruit_TinyUSB.h> // Richiesto da entrambi
#include <Wire.h>              // Richiesto per IMU
#include "SparkFun_BMI270_Arduino_Library.h" // Richiesto per IMU


// PARAMETRI AUDIO Microfono
// ===================================================
const int micPin = A0;     // Pin microfono
const int DC_OFFSET = 631; // Offset DC (DA RICALIBRARE)
const float GAIN_FACTOR = 3.0;
const int Gamma = 30;      // Soglia rumore (DA RICALIBRARE)

// Parametri di campionamento (fs = 2kHz)
const int fs = 2000;
const unsigned long samplePeriod_us = 1000000 / fs;
unsigned long lastSampleTime = 0;

// Parametri Frame (per ZCR & Energia)
const int frame_ms = 25;
const int frame_len = fs * frame_ms / 1000;
int16_t sigBuffer[frame_len];
int bufferIndex = 0;

// Filtro e Peak Detection
const int HISTORY_SIZE = 10;
const int SMOOTH_WINDOW = 3;

// Buffer per ZCR
float zcrHistory[HISTORY_SIZE];
int zcrHistoryIndex = 0;
float smoothedZCR = 0.0;

// Buffer per ENERGIA
float energyHistory[HISTORY_SIZE];
int energyHistoryIndex = 0;
float smoothedEnergy = 0.0;

// *** SOGLIE CHIAVE AUDIO DA RICALIBRARE! ***
const float ZCR_COUGH_THRESHOLD = 0.1;
const float ENERGY_COUGH_THRESHOLD = 100.0; // DA RICALIBRARE!

// ===================================================
// PARAMETRI IMU ***da ricalibrare***
// ===================================================
#define TIME_INTGR 100
#define SOGLIA_TOSSE 0.25

// Oggetto sensore IMU
BMI270 imu;
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR; // 0x68

// Variabili per calcoli IMU
double a_x, a_y, a_z, d_x, d_y, d_z, delta, intgr_z = 0;
long t_old, timer = 0;

// VARIABILI PER IL LATCH TEMPORIZZATO IMU (resta true per più tempo)
const long IMU_PEAK_DURATION_MS = 500; // Tieni 'true' per 0.5 secondi (possiamo anche aumentare)
unsigned long imuPeakStartTime = 0;    // Timer per la durata del picco

// ===================================================
// STATI GLOBALI PER LA LOGICA COMBINATA
// ===================================================
bool mic_isPeak = false;  // Stato rilevamento picco da microfono
bool imu_isTosse = false; // Stato rilevamento tosse da IMU

// --- NUOVE VARIABILI PER IL CONTEGGIO ---
unsigned int coughCount = 0;  // Il nostro contatore totale
bool isCountingCough = false; // Variabile "latch" per contare la tosse solo una volta

// ===================================================
// Funzioni per ZCR e ENERGIA (per microfono)
// ===================================================

float computeZCR(int16_t* frame, int len) {
  int signChanges = 0;
  for (int i = 1; i < len; i++) {
    if ((long)frame[i] * (long)frame[i - 1] < 0) {
      signChanges++;
    }
  }
  return (float)signChanges / (float)(len - 1);
}

float computeEnergy(int16_t* frame, int len) {
  unsigned long sum = 0;
  for (int i = 0; i < len; i++) {
    sum += abs(frame[i]);
  }
  return (float)sum / (float)len;
}

// ===================================================
// SETUP (Combinato)
// ===================================================
void setup() {
  // --- Setup Seriale ---
  Serial.begin(115200);
  unsigned long t0_serial = millis();
  while (!Serial && millis() - t0_serial < 3000) {
    // Attendi la connessione seriale (max 3 sec)
  }
  Serial.println("Avvio... Setup combinato Microfono + IMU.");

  // Setup IMU
  Wire.begin();
  Serial.println("Inizializzazione IMU BMI270...");
  while (imu.beginI2C(i2cAddress) != BMI2_OK) {
    Serial.println("Errore: BMI270 non connesso, controllare cablaggio e indirizzo I2C!");
    delay(1000);
  }
  Serial.println("BMI270 connesso!");
  
  // Inizializza t_old per la logica IMU
  t_old = millis();
  timer = millis();

  //Setup Audio
  // Inizializza buffer di cronologia
  for (int i = 0; i < HISTORY_SIZE; i++) {
    zcrHistory[i] = 0.0;
    energyHistory[i] = 0.0;
  }
  
  Serial.println("Setup completato. Avvio loop principale.");
}

// ===================================================
// MAIN LOOP (Combinato)
// ===================================================
void loop() {
  // ===================================================
// 1. LOGICA IMU (con Latch Temporizzato)
// ===================================================

imu.getSensorData(); // Leggi sempre i dati nuovi

delta = millis() - t_old;
t_old = millis();

d_z = (imu.data.accelZ - a_z) / delta;
a_z = imu.data.accelZ;

intgr_z += abs(d_z);

// --- A. Logica di START Picco (ogni 100ms) ---
// Controlla l'integrale IMU ogni TIME_INTGR
if (millis() - timer >= TIME_INTGR) {

  // Se l'integrale supera la soglia E non c'è già un picco attivo...
  if ((intgr_z > SOGLIA_TOSSE) && (!imu_isTosse)) {
    // ... questo è un NUOVO picco IMU.
    imu_isTosse = true;
    imuPeakStartTime = millis(); // Avvia il timer di durata
  }
  // Azzera integrale e ri-inizializza il timer
  timer = millis();
  intgr_z = 0;
}

// --- B. Logica di STOP Picco (continua) ---
// Controlla se il picco è "scaduto"
if (imu_isTosse) {
  // Se è passato troppo tempo dall'inizio del picco...
  if (millis() - imuPeakStartTime >= IMU_PEAK_DURATION_MS) {
    // ...resetta lo stato, pronto per un nuovo rilevamento.
    imu_isTosse = false;
  }
}
  // 2. ESEGUI LOGICA AUDIO 
  // Questa è la parte critica per il tempo (campionamento)
  
  if (micros() - lastSampleTime >= samplePeriod_us) {
    lastSampleTime += samplePeriod_us;

    // 1. Leggi, centra, applica soglia e guadagno 
    int rawVal = analogRead(micPin);
    int sig_centered = rawVal - DC_OFFSET;
    if (abs(sig_centered) < Gamma) {
      sig_centered = 0;
    }
    int16_t sig_gained = (int16_t)(sig_centered * GAIN_FACTOR);

    // 2. Salva nel buffer
    sigBuffer[bufferIndex++] = sig_gained;

    // 3. ELABORAZIONE FRAME
    // Questo blocco si attiva ogni 25ms (frame_len campioni)
    if (bufferIndex >= frame_len) {

      // Calcola ZCR e Energia del frame corrente
      float currentZCR = computeZCR(sigBuffer, frame_len);
      float currentEnergy = computeEnergy(sigBuffer, frame_len);
      bufferIndex = 0;

      // Aggiorna buffer di cronologia
      zcrHistory[zcrHistoryIndex] = currentZCR;
      energyHistory[energyHistoryIndex] = currentEnergy;

      // Calcola medie mobili (smoothing)
      smoothedZCR = 0.0;
      smoothedEnergy = 0.0;
      for (int i = 0; i < SMOOTH_WINDOW; i++) {
        int idx_zcr = (zcrHistoryIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
        smoothedZCR += zcrHistory[idx_zcr];
        int idx_energy = (energyHistoryIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
        smoothedEnergy += energyHistory[idx_energy];
      }
      smoothedZCR /= (float)SMOOTH_WINDOW;
      smoothedEnergy /= (float)SMOOTH_WINDOW;

      // Logica di Peak Detection (aggiorna lo stato globale 'mic_isPeak')
      if (smoothedZCR > ZCR_COUGH_THRESHOLD && smoothedEnergy > ENERGY_COUGH_THRESHOLD) {
        if (!mic_isPeak) {
          // Serial.println(">>> PICCO MIC RILEVATO <<<"); // Debug (opzionale)
          mic_isPeak = true;
        }
      } else {
        if (mic_isPeak) {
          // Serial.println("--- Fine picco mic ---"); // Debug (opzionale)
          mic_isPeak = false;
        }
      }

      // --- 4. LOGICA COMBINATA, CONTEGGIO E STAMPA PLOTTER ---
      
      // Controlla se ENTRAMBI i sistemi hanno rilevato la tosse
      bool combined_cough = mic_isPeak && imu_isTosse;

      // Logica di CONTEGGIO (rileva la presenza di una tosse)
      if (combined_cough) {
        // Se i sensori rilevano la tosse E non la stavamo già contando...
        if (!isCountingCough) {
          // ...questo è un NUOVO evento di tosse.
          coughCount++; // Incrementa il contatore
          isCountingCough = true; // Imposta il "latch" per non contare di nuovo
        }
        // Se combined_cough è true e isCountingCough è true, significa
        // che siamo ancora nello stesso colpo di tosse, quindi non facciamo nulla.
        
      } else {
        // Se non c'è tosse, resettiamo il "latch".
        // Siamo pronti a rilevare il prossimo colpo di tosse.
        isCountingCough = false;
      }

      // Stampa il CONTEGGIO attuale per il Serial Plotter
      // Il grafico mostrerà una linea che sale a "scalini"
      Serial.println(coughCount);

      // Avanza gli indici della cronologia
      zcrHistoryIndex = (zcrHistoryIndex + 1) % HISTORY_SIZE;
      energyHistoryIndex = (energyHistoryIndex + 1) % HISTORY_SIZE;
    }
  }
} // Fine loop