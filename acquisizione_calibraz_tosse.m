%% DATA LOGGER PER CALIBRAZIONE TOSSE (GROUND TRUTH)
% Questo script acquisisce dati da Arduino e permette l'etichettatura manuale.
% 
% OUTPUT: File 'dataset_tosse.csv' con colonne:
% [Timestamp, ZCR, Energy, IMU_Jerk, Manual_Label]
%
% ISTRUZIONI:
% 1. Avvia lo script.
% 2. Tieni premuta la BARRA SPAZIATRICE quando tossisci (Label = 1).
% 3. Rilascia quando non tossisci (Label = 0).
% 4. Chiudi la finestra del grafico per terminare e salvare.

%% DATA LOGGER DUAL RATE (Audio 25ms / IMU Fast)
clear; clc; close all;

%% 1. CONFIGURAZIONE
serialPort = "COM3";   % <--- CONTROLLA LA TUA PORTA
baudRate = 500000;     % <--- IMPORTANTE: Deve corrispondere ad Arduino
outputFileName = "dataset_tosse.csv";

%% 2. PREPARAZIONE GRAFICI (APERTURA IMMEDIATA)
% Apriamo la finestra prima della connessione per evitare che sembri bloccato
global isLabelActive; isLabelActive = 0; 

disp('Apertura finestra grafici...');
fig = figure('Name', 'DUAL RATE LOGGER - Connessione in corso...', 'Color', 'w');
set(fig, 'WindowKeyPressFcn', @(~,e) keySwitch(e, 1));
set(fig, 'WindowKeyReleaseFcn', @(~,e) keySwitch(e, 0));

subplot(3,1,1); hZCR = animatedline('Color', 'm'); title('ZCR (A gradini)'); grid on; ylim([0 1]);
subplot(3,1,2); hEnergy = animatedline('Color', 'b'); title('Energia (A gradini)'); grid on;
subplot(3,1,3); hIMU = animatedline('Color', 'r'); 
hold on; hLabel = animatedline('Color', 'k', 'LineWidth', 2); hold off;
title('Jerk (Continuo) & Label'); legend('Jerk', 'Label'); grid on;

drawnow; % Forza il disegno immediato della finestra

%% 3. CONNESSIONE
disp('Connessionead Arduino...');
clear s; % Pulisce l'oggetto serialport precedente

try
    s = serialport(serialPort, baudRate);
    configureTerminator(s, "CR/LF");
    flush(s);
    s.Timeout = 0.5; 
    disp('CONNESSO! Ricezione ad alta frequenza...');
    fig.Name = 'DUAL RATE LOGGER - CONNESSO'; % Aggiorna titolo
catch ME
    close(fig); % Chiude la finestra se fallisce
    error(['Errore Connessione: ' ME.message '. Controlla la porta COM e chiudi Arduino IDE.']);
end

% Buffer dati
dataLog = zeros(100000, 5); 
idx = 1;
startTime = datetime('now');

disp('--- REGISTRAZIONE AVVIATA ---');

%% 4. LOOP VELOCE
while isvalid(fig)
    try
        if s.NumBytesAvailable > 0
            line = readline(s);
            if isempty(line), continue; end
            
            strData = split(line, ',');
            vals = str2double(strData);
            
            if length(vals) >= 3 && ~any(isnan(vals))
                tElapsed = seconds(datetime('now') - startTime);
                
                % Salvataggio in RAM 
                if idx <= size(dataLog, 1)
                    dataLog(idx, :) = [tElapsed, vals(2), vals(1), vals(3), isLabelActive];
                    idx = idx + 1;
                end
                
                % Aggiornamento Grafico (Decimato per fluidità)
                if mod(idx, 20) == 0
                    addpoints(hZCR, tElapsed, vals(2));
                    addpoints(hEnergy, tElapsed, vals(1));
                    addpoints(hIMU, tElapsed, vals(3));
                    addpoints(hLabel, tElapsed, isLabelActive * max(10, vals(3)));
                    
                    if tElapsed > 5 
                       ax1=subplot(3,1,1); xlim([tElapsed-5, tElapsed]);
                       ax2=subplot(3,1,2); xlim([tElapsed-5, tElapsed]);
                       ax3=subplot(3,1,3); xlim([tElapsed-5, tElapsed]);
                    end
                    drawnow limitrate;
                end
            end
        else
            % IMPORTANTE: Se non ci sono dati, fai respirare la CPU
            % Senza questo, MATLAB può congelare l'interfaccia grafica
            pause(0.005); 
        end
    catch
        break;
    end
end

%% 5. SALVATAGGIO
if idx > 1
    disp('Salvataggio CSV...');
    T = array2table(dataLog(1:idx-1, :), 'VariableNames', {'Time', 'ZCR', 'Energy', 'Jerk', 'Label'});
    writetable(T, outputFileName);
    fprintf('Salvati %d campioni in: %s\n', idx-1, outputFileName);
end

function keySwitch(event, state)
    global isLabelActive;
    if strcmp(event.Key, 'space'), isLabelActive = state; end
end