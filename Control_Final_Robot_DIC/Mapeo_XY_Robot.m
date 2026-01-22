%% Script 2: Leer puntos del cuadrado, hacer IK y generar G-code en TXT

clear; clc;

%% ----- PARÁMETROS DEL ROBOT -----
L1 = 12;   % cm
L2 = 12;   % cm

% Rama del codo: true = "codo arriba", false = "codo abajo"
elbowUp = true;

%% ----- PARÁMETROS DE VELOCIDAD (FÁCIL DE MODIFICAR) -----
speedTheta1 = 20;  % para Y en el G-code (motor 1)
speedTheta2 = 20;  % para Z en el G-code (motor 2)

%% ----- ARCHIVOS -----
inputFile  = 'coordenadas_xy_cm_opt.txt';  % archivo de puntos (x y)
outputFile = 'coords15AngularOut.txt'; % archivo de salida G-code

%% Leer puntos (x y) desde el TXT
data = load(inputFile);  % asume dos columnas: x y

if size(data, 2) ~= 2
    error('El archivo %s debe tener exactamente dos columnas (x y).', inputFile);
end

numPoints   = size(data, 1);
theta1_deg  = zeros(numPoints, 1);
theta2_deg  = zeros(numPoints, 1);
reachable   = true(numPoints, 1);

%% Calcular IK para cada punto
for k = 1:numPoints
    % ============================================================
    % INTERCAMBIO DE EJES:
    %  - Antes: x = data(k,1); y = data(k,2);
    %  - Ahora: X+ toma lo que era Y+, e Y+ toma lo que era X+
    % ============================================================
    x = data(k, 2);  % X toma la antigua columna Y
    y = data(k, 1);  % Y toma la antigua columna X

    [th1, th2, ok] = ik_2R_point(x, y, L1, L2, elbowUp);

    if ~ok
        fprintf('Punto (%.2f, %.2f) fuera de alcance. Se marcará como comentario.\n', x, y);
        reachable(k)  = false;
        theta1_deg(k) = NaN;
        theta2_deg(k) = NaN;
    else
        theta1_deg(k) = th1;
        theta2_deg(k) = th2;
    end
end

%% Guardar G-code en archivo TXT (bloque original)
fid = fopen(outputFile, 'w');
if fid == -1
    error('No se pudo abrir el archivo %s para escritura.', outputFile);
end

for k = 1:numPoints
    % Aquí data(k,:) ya no se usa para la IK, pero lo dejamos igual
    if ~reachable(k)
        % Comentario de punto fuera de alcance (por si acaso)
        fprintf(fid, '; Punto fuera de alcance (%.2f, %.2f)\n', data(k,1), data(k,2));
        continue;
    end

    % Nomenclatura:
    %   G1 X(theta1) A(theta2) Y(vel motor 1) Z(vel motor 2)
    fprintf(fid, 'G1 X%.2f A%.2f Y%.2f Z%.2f\n', ...
        theta1_deg(k), theta2_deg(k), speedTheta1, speedTheta2);
end

%%///////////////////////////////////////////////////////////

%% ----- GENERAR G-CODE CON M3/M5 Y SALTOS DE RENGLÓN -----
% Este bloque asume:
%   - inputFile: archivo de entrada con coordenadas (puede tener líneas en blanco)
%   - outputFile: archivo de salida para el G-code
%   - theta1_deg, theta2_deg: ángulos ya calculados por IK
%   - reachable: vector lógico indicando si el punto es alcanzable
%   - speedTheta1, speedTheta2: velocidades (Y y Z en el G-code)
%
% Convención:
%   M5 -> subir plumón  (NO pinta)
%   M3 -> bajar plumón  (SÍ pinta)

fidIn  = fopen(inputFile,  'r');
if fidIn < 0
    error('No se pudo abrir el archivo de entrada: %s', inputFile);
end

fidOut = fopen(outputFile, 'w');
if fidOut < 0
    fclose(fidIn);
    error('No se pudo abrir el archivo de salida: %s', outputFile);
end

% --- Estado inicial ---
% El robot arranca en X=0, A=0 y plumón abajo mecánicamente,
% pero lógicamente vamos a arrancar levantando el plumón con M5.
fprintf(fidOut, "M5\n");  % Subir plumón al inicio (no pinta)

pointIdx       = 1;   % índice sobre theta1_deg/theta2_deg
firstStroke    = true;
needLiftTravel = false;  % se pone true cuando se detecta una línea en blanco
penIsDown      = false;  % estado lógico de la pluma en el G-code

lineNumber = 0;

while true
    tline = fgetl(fidIn);
    if ~ischar(tline)
        break;  % fin del archivo
    end
    lineNumber = lineNumber + 1;

    str = strtrim(tline);

    % --- Línea en blanco = salto de renglón / nuevo trazo ---
    if isempty(str)
        % Marcamos que el siguiente punto que venga será el inicio de un trazo nuevo
        needLiftTravel = true;
        continue;
    end

    % Aquí la línea SÍ tiene texto, asumimos "x y"
    if pointIdx > numPoints
        warning('Hay más líneas con coordenadas que puntos calculados (línea %d).', lineNumber);
        break;
    end

    if ~reachable(pointIdx)
        % Punto fuera de alcance: lo saltamos (opcional: comentar en el G-code)
        % fprintf(fidOut, ";; Punto fuera de alcance en línea %d del TXT\n", lineNumber);
        pointIdx = pointIdx + 1;
        continue;
    end

    th1 = theta1_deg(pointIdx);
    th2 = theta2_deg(pointIdx);

    % ---------- LÓGICA DE TRAZOS ----------

    if firstStroke
        % Primer punto de TODO el dibujo:
        % Ya hicimos M5 al inicio -> viajar a este primer punto SIN pintar
        fprintf(fidOut, "G1 X%.2f A%.2f Y%.2f Z%.2f\n", ...
                th1, th2, speedTheta1, speedTheta2);
        % Ahora bajamos plumón para empezar a pintar desde el siguiente segmento
        fprintf(fidOut, "M3\n");
        penIsDown   = true;
        firstStroke = false;

    else
        if needLiftTravel
            % Acabamos de pasar por una línea en blanco:

            % 1) Subir plumón
            if penIsDown
                fprintf(fidOut, "M5\n");
                penIsDown = false;
            else
                % Por si acaso ya estaba arriba
                fprintf(fidOut, "M5\n");
            end

            % 2) Viaje hasta el nuevo punto SIN pintar
            fprintf(fidOut, "G1 X%.2f A%.2f Y%.2f Z%.2f\n", ...
                    th1, th2, speedTheta1, speedTheta2);

            % 3) Bajar plumón para seguir pintando desde aquí
            fprintf(fidOut, "M3\n");
            penIsDown      = true;
            needLiftTravel = false;

        else
            % Punto normal dentro del mismo trazo: simplemente dibujamos
            fprintf(fidOut, "G1 X%.2f A%.2f Y%.2f Z%.2f\n", ...
                    th1, th2, speedTheta1, speedTheta2);
        end
    end

    pointIdx = pointIdx + 1;
end

% ---------- FINAL DEL DIBUJO ----------
% Al terminar todos los puntos:

% 1) Subimos plumón
if penIsDown
    fprintf(fidOut, "M5\n");
else
    fprintf(fidOut, "M5\n");  % nos aseguramos
end

% 2) Regresamos a la posición de origen (X=0, A=0) sin pintar
fprintf(fidOut, "G1 X0.00 A0.00 Y%.2f Z%.2f\n", speedTheta1, speedTheta2);

% 3) Bajamos plumón para que descanse en su lugar
fprintf(fidOut, "M3\n");

fclose(fidIn);
fclose(fidOut);

disp(['G-code generado en: ', outputFile]);

%%////////////////////////////////////////////////////////////////////////////////////////////////////////

fclose(fid);
fprintf('Listo. Se generó el archivo %s con el G-code.\n', outputFile);

%% ---------------- FUNCIÓN DE CINEMÁTICA INVERSA ----------------
function [theta1_deg, theta2_deg, isReachable] = ik_2R_point(x, y, L1, L2, elbowUp)
% x, y en cm
% L1, L2 en cm
% elbowUp = true para una rama, false para la otra
% Devuelve ángulos en grados y bandera de alcanzabilidad

r2 = x^2 + y^2;

% Ley de cosenos para theta2
cosTheta2 = (r2 - L1^2 - L2^2) / (2 * L1 * L2);
if abs(cosTheta2) > 1 + 1e-9
    theta1_deg   = NaN;
    theta2_deg   = NaN;
    isReachable  = false;
    return;
end

% Corregir pequeñas desviaciones numéricas
cosTheta2 = max(min(cosTheta2, 1), -1);
sinTheta2_abs = sqrt(max(0, 1 - cosTheta2^2));

if elbowUp
    sinTheta2 = sinTheta2_abs;
else
    sinTheta2 = -sinTheta2_abs;
end

theta2 = atan2(sinTheta2, cosTheta2);

% Theta1
phi  = atan2(y, x);
beta = atan2(L2 * sinTheta2, L1 + L2 * cosTheta2);
theta1 = phi - beta;

theta1_deg  = rad2deg(theta1);
theta2_deg  = rad2deg(theta2);
isReachable = true;
end
