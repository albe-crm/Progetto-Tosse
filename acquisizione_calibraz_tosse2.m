%% LOGGER CON CONTATORE MANUALE (Basato su Dual Rate Fast)
% ISTRUZIONI:
% 1. Avvia lo script.
% 2. Quando tossisci, dai un COLPO SECCO (Tap) sulla BARRA SPAZIATRICE.
% 3. Il contatore a video salirà.
% 4. Chiudi il grafico per salvare.

clear; clc; close all;

%% 1. CONFIGURAZIONE
serialPort = "COM3";   % <--- CONTROLLA LA TUA PORTA
baudRate = 500000;     % <--- UGUALE AL TUO CODICE
outputFileName = "dataset_validazione2.csv";

%% 2. PREPARAZIONE GRAFICI (APERTURA IMMEDIATA)
global manualCount; manualCount = 0; % Contatore globale
global triggerVisual; triggerVisual = 0; % Per l'impulso grafico

disp('Apertura finestra grafici...');
fig = figure('Name', 'VALIDAZIONE - Connessione in corso...', 'Color', 'w');

% CALLBACK MODIFICATA: Conta ogni pressione
set(fig, 'WindowKeyPressFcn', @keyPressCounter);

subplot(3,1,1); hZCR = animatedline('Color', 'm'); title('ZCR'); grid on; ylim([0 1]);
subplot(3,1,2); hEnergy = animatedline('Color', 'b'); title('Energia'); grid on;
subplot(3,1,3); hIMU = animatedline('Color', 'r'); 
hold on; hClick = animatedline('Color', 'k', 'LineWidth', 2); hold off;
title('Jerk & Click Manuali'); legend('Jerk', 'Click'); grid on;

% NUOVO: Etichetta testuale per vedere il numero
lbl = annotation('textbox', [0.1, 0.92, 0.8, 0.08], 'String', 'CONTEGGIO: 0', ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
    'FontSize', 14, 'FontWeight', 'bold', 'Color', [0 0.5 0]);

drawnow; 

%% 3. CONNESSIONE
disp('Connessione ad Arduino...');
if ~isempty(instrfind), delete(instrfind); end
clear s;

try
    s = serialport(serialPort, baudRate);
    configureTerminator(s, "CR/LF");
    flush(s);
    s.Timeout = 0.5; 
    disp('CONNESSO! Modalità Contatore.');
    fig.Name = 'VALIDAZIONE - PRONTO'; 
catch ME
    close(fig);
    error(['Errore Connessione: ' ME.message]);
end

dataLog = zeros(100000, 5); 
idx = 1;
startTime = datetime('now');

%% 4. LOOP VELOCE (Identico al tuo)
while isvalid(fig)
    try
        if s.NumBytesAvailable > 0
            line = readline(s);
            if isempty(line), continue; end
            
            vals = str2double(split(line, ','));
            
            if length(vals) >= 3 && ~any(isnan(vals))
                tElapsed = seconds(datetime('now') - startTime);
                
                % Salvataggio: Colonna 5 è il CONTATORE TOTALE
                if idx <= size(dataLog, 1)
                    dataLog(idx, :) = [tElapsed, vals(2), vals(1), vals(3), manualCount];
                    idx = idx + 1;
                end
                
                % Aggiornamento Grafico
                if mod(idx, 20) == 0
                    addpoints(hZCR, tElapsed, vals(2));
                    addpoints(hEnergy, tElapsed, vals(1));
                    addpoints(hIMU, tElapsed, vals(3));
                    
                    % Gestione impulso visivo (linea nera che salta e torna giù)
                    if triggerVisual > 0
                        addpoints(hClick, tElapsed, max(10, vals(3))); 
                        triggerVisual = triggerVisual - 1; 
                    else
                        addpoints(hClick, tElapsed, 0);
                    end
                    
                    % Aggiorna il testo in alto
                    lbl.String = ['CONTEGGIO: ' num2str(manualCount)];
                    
                    if tElapsed > 5 
                       ax1=subplot(3,1,1); xlim([tElapsed-5, tElapsed]);
                       ax2=subplot(3,1,2); xlim([tElapsed-5, tElapsed]);
                       ax3=subplot(3,1,3); xlim([tElapsed-5, tElapsed]);
                    end
                    drawnow limitrate;
                end
            end
        else
            pause(0.005); 
        end
    catch
        break;
    end
end

%% 5. SALVATAGGIO
if idx > 1
    disp('Salvataggio CSV...');
    % Nota il nome dell'ultima colonna: Manual_Count_Total
    T = array2table(dataLog(1:idx-1, :), 'VariableNames', {'Time', 'ZCR', 'Energy', 'Jerk', 'Manual_Count_Total'});
    writetable(T, outputFileName);
    fprintf('FILE SALVATO: %s\n', outputFileName);
    fprintf('Hai contato in totale: %d colpi.\n', manualCount);
end

%% FUNZIONE PRESSIONE TASTO (MODIFICATA)
function keyPressCounter(~, event)
    global manualCount;
    global triggerVisual;
    % Incrementa solo se premi SPAZIO
    if strcmp(event.Key, 'space')
        manualCount = manualCount + 1;
        triggerVisual = 5; % Fa durare il picco nero per 5 aggiornamenti grafici
    end
end