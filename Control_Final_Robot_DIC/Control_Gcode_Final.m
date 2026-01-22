%% === Script principal: enviar G-code al Arduino por serial ===
clear; clc; close all;

%% === Parámetros de conexión y archivo ===
portName  = "COM3";          % Cambia al puerto de tu Arduino
baudRate  = 115200;          % Debe coincidir con Serial.begin del Arduino
gcodeFile = "coords15AngularOut.txt";% Archivo de G-code a enviar

%% === Abrir puerto serie ===
% Si ya existe un objeto serie previo en el workspace, lo limpiamos
if exist("s", "var")
    try
        clear s;
    end
end

% Crear objeto serialport
% Usamos un Timeout grande por si algún movimiento tarda varios segundos
s = serialport(portName, baudRate, "Timeout", 60);  % 60 s de margen
configureTerminator(s, "LF");                       % Arduino usa '\n' en Serial.println

% Pequeña pausa por si el Arduino se resetea al abrir el puerto
pause(2);

% Limpiar buffers de entrada/salida
flush(s);

fprintf("Conectado a %s a %d baudios.\n", portName, baudRate);

%% === Abrir archivo de G-code ===
fid = fopen(gcodeFile, 'r');
if fid == -1
    error("No se pudo abrir el archivo: %s", gcodeFile);
end
fprintf("Leyendo G-code desde: %s\n", gcodeFile);

%% === Enviar línea por línea ===
lineNumber = 0;

while true
    tline = fgetl(fid);
    if ~ischar(tline)  % Fin de archivo
        break;
    end

    % Limpiar espacios al inicio/fin
    tline = strtrim(tline);

    % Saltar líneas vacías o comentarios (ajusta según tu formato)
    if isempty(tline)
        continue;
    end
    if startsWith(tline, ";") || startsWith(tline, "%") || startsWith(tline, "#")
        continue;
    end

    lineNumber = lineNumber + 1;

    % Enviar la línea al Arduino
    writeline(s, tline);

    % Esperar respuesta "ok" del Arduino
    try
        resp = strtrim(readline(s));
    catch ME
        warning("Timeout o error leyendo respuesta en línea %d: %s", ...
                 lineNumber, ME.message);
        resp = "";
    end

    % Mensaje opcional para monitorear
    fprintf("Línea %5d: %-40s | Arduino: %s\n", lineNumber, tline, resp);

    % Si quieres ser estricto con el "ok", puedes descomentar:
    % if ~strcmpi(resp, "ok")
    %     warning("Respuesta inesperada en línea %d: %s", lineNumber, resp);
    % end
end

%% === Cerrar archivo y puerto ===
fclose(fid);
clear s;

fprintf("Envío de G-code terminado. Líneas procesadas: %d\n", lineNumber);
