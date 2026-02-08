clear; clc; close all;
%% 1. CONFIGURAZIONE
serialPort = "COM37";   % <--- CONTROLLA LA TUA PORTA
baudRate = 500000;     % <--- UGUALE AL TUO CODICE
outputFileName = "dataset_validazione2.csv";
%% 2. CONNESSIONE
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
global manualCount; manualCount = 0; % Contatore globale 
fig = figure('Name', 'VALIDAZIONE - Connessione in corso...', 'Color', 'w');
set(fig, 'WindowKeyPressFcn', @keyPressCounter);
while isvalid(fig) 
    try
        if s.NumBytesAvailable > 0
            line = readline(s);
            line
            manualCount
            if isempty(line), continue; end
            
            vals = str2double(split(line, ';'));
            
            if length(vals) >= 3 && ~any(isnan(vals))
                tElapsed = seconds(datetime('now') - startTime);
                
                % Salvataggio: Colonna 5 è il CONTATORE TOTALE
                if idx <= size(dataLog, 1)
                    dataLog(idx, :) = [tElapsed, vals(2), vals(1), vals(3), manualCount];
                    idx = idx + 1;
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
    end
end