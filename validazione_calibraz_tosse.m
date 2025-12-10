%% 2. ANALIZZATORE E VALIDATORE (Post-Processing)
% Questo script legge il CSV registrato e simula l'algoritmo di Arduino.
% Confronta il conteggio dell'algoritmo con il tuo conteggio manuale.

clear; clc; close all;

%% 1. CARICAMENTO DATI
fileName = "dataset_validazione2.csv";

if ~isfile(fileName)
    error('File CSV non trovato! Esegui prima il Logger Contatore.');
end

T = readtable(fileName);
time = T.Time;
raw_zcr = T.ZCR;
raw_energy = T.Energy;
raw_jerk = T.Jerk;
manual_count_trend = T.Manual_Count_Total;

% Il numero reale di colpi di tosse (l'ultimo valore del contatore)
REAL_COUGHS = max(manual_count_trend);

%% 2. PARAMETRI DA TESTARE (Modifica questi per trovare le soglie perfette!)
% ---------------------------------------------------------
TH_ZCR = 0.01;         % Soglia ZCR
TH_ENERGY = 115.0;     % Soglia Energia
TH_JERK = 0.15;        % Soglia Jerk (Scossa secca)
% ---------------------------------------------------------

% Costanti fisse di Arduino
IMU_LATCH_MS = 500;    % Durata Latch IMU (ms)
SMOOTH_WIN = 3;        % Finestra media mobile Audio

%% 3. PRE-PROCESSING (Uguale ad Arduino)
% Arduino fa la media degli ultimi 3 valori. Facciamolo anche qui.
smooth_zcr = movmean(raw_zcr, [SMOOTH_WIN-1, 0]); % [Indietro, Avanti]
smooth_energy = movmean(raw_energy, [SMOOTH_WIN-1, 0]);

%% 4. SIMULAZIONE LOOP ARDUINO
algo_count = 0;
is_counting = false;       % Stato per evitare doppi conteggi
imu_latch_timer = -9999;   % Timer per il latch
algo_debug_plot = zeros(height(T), 1); % Per il grafico verde

for i = 1:height(T)
    
    % --- LOGICA IMU (Con Latch) ---
    % Se il Jerk supera la soglia, resetto il timer del latch
    if raw_jerk(i) > TH_JERK
        imu_latch_timer = time(i); 
    end
    
    % Controllo se il latch è ancora attivo (tempo attuale - tempo trigger < 0.5s)
    imu_is_active = (time(i) - imu_latch_timer) <= (IMU_LATCH_MS / 1000);
    
    % --- LOGICA AUDIO ---
    mic_is_peak = (smooth_zcr(i) > TH_ZCR) && (smooth_energy(i) > TH_ENERGY);
    
    % --- FUSIONE SENSORI ---
    combined_cough = mic_is_peak && imu_is_active;
    
    % --- CONTEGGIO (Rising Edge) ---
    if combined_cough
        algo_debug_plot(i) = 1; % Segna rilevamento per il grafico
        
        if ~is_counting
            algo_count = algo_count + 1; % +1 Tosse Rilevata
            is_counting = true;
        end
    else
        is_counting = false; % Reset quando scende sotto soglia
    end
end

%% 5. RISULTATI
fprintf('\n============================================\n');
fprintf('REPORT VALIDAZIONE SOGLIE\n');
fprintf('============================================\n');
fprintf('Soglie Testate: ZCR=%.3f | Energy=%.1f | Jerk=%.2f\n', TH_ZCR, TH_ENERGY, TH_JERK);
fprintf('--------------------------------------------\n');
fprintf('Tosse REALE (Tuoi Click):     %d\n', REAL_COUGHS);
fprintf('Tosse RILEVATA (Algoritmo):   %d\n', algo_count);
fprintf('--------------------------------------------\n');

delta = algo_count - REAL_COUGHS;
if delta == 0
    fprintf('ECCELLENTE! Le soglie sono perfette.\n');
elseif delta > 0
    fprintf('ATTENZIONE: Troppi rilevamenti (+%d). Alza le soglie.\n', delta);
else
    fprintf('ATTENZIONE: Tosse persa (-%d). Abbassa le soglie.\n', abs(delta));
end
fprintf('============================================\n');

%% 6. GRAFICO ANALISI
figure('Name', 'Analisi Soglie', 'Color', 'w');

ax1 = subplot(3,1,1);
plot(time, smooth_energy, 'b'); yline(TH_ENERGY, 'b--', 'LineWidth', 2);
title('Audio Energia (Smoothed)'); grid on; legend('Segnale', 'Soglia');

ax2 = subplot(3,1,2);
plot(time, raw_jerk, 'r'); yline(TH_JERK, 'r--', 'LineWidth', 2);
title('IMU Jerk'); grid on; legend('Segnale', 'Soglia');

ax3 = subplot(3,1,3);
% Disegna il conteggio manuale (linea nera)
plot(time, manual_count_trend, 'k', 'LineWidth', 1.5); hold on;
% Disegna i rilevamenti dell'algoritmo (area verde)
area(time, algo_debug_plot * max(manual_count_trend), 'FaceColor', 'g', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
title('Confronto Finale'); legend('Conteggio Manuale', 'Rilevamento Algo'); grid on;

linkaxes([ax1, ax2, ax3], 'x');