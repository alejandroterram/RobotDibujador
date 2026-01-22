% === 03: Exportar coordenadas XY en cm a un archivo TXT (orden optimizado) ===
% Requiere:
%   - 'trayectorias.mat' generado por el script de trayectorias
%
% Funciones:
%   1) Lee strokes_cm (trayectorias en cm)
%   2) Opcional: re-muestrea cada trazo con un paso "paso_cm"
%   3) Aplica rotación y offsets globales
%   4) Reordena los trazos para que el robot vaya al trazo más cercano
%      (y decide si conviene recorrer cada trazo al derecho o al revés)
%   5) Genera un TXT con dos columnas: X_cm  Y_cm
%      y un renglón en blanco entre trazos (NO fusiona trazos)
%
% Uso típico:
%   1) run preprocesar_imagen_sketch.m
%   2) run 02_linearizar_imagen.m
%   3) run Process_Pict_Pt3.m   (IK/Trayectorias)
%   4) run Process_Pict_Pt4.m   (este)

clear; clc; close all;

%% 0) PARÁMETROS DE EXPORTACIÓN Y CALIBRACIÓN

nombreArchivoTXT = 'coordenadas_xy_cm_opt.txt';

% --- Paso entre puntos a lo largo del trazo (en cm) ---
% Si paso_cm <= 0: NO se re-muestrea, se usan puntos originales.
% Si paso_cm  > 0: se re-muestrea para que la distancia entre puntos
%                  consecutivos sea ~ paso_cm.
paso_cm = 0.05;   % 0.1 -> 1 mm, 0.2 -> 2 mm, 0.5 -> 5 mm, etc.

% --- Transformaciones geométricas globales ---
angulo_deg = 180;    % Rotación global en grados (CCW)
offsetX_cm = 5.5;    % Offset global en X (cm)
offsetY_cm = 23;    % Offset global en Y (cm)

%% 1) Cargar trayectorias

datos = load('trayectorias.mat');

if ~isfield(datos, 'strokes_cm')
    error('trayectorias.mat no contiene la variable strokes_cm.');
end

strokes_cm       = datos.strokes_cm;
cmPorPx          = datos.cmPorPx;
minStrokeLength  = datos.minStrokeLength;

numStrokes = numel(strokes_cm);

fprintf('Trayectorias cargadas: %d strokes\n', numStrokes);
fprintf('cmPorPx = %.4f, minStrokeLength = %d\n', cmPorPx, minStrokeLength);

%% 2) Re-muestrear (opcional) + rotar + desplazar cada trazo

strokes_base_X = cell(numStrokes, 1);
strokes_base_Y = cell(numStrokes, 1);

for k = 1:numStrokes
    pts = strokes_cm{k};   % [N x 2] -> [x_cm, y_cm]
    if isempty(pts)
        strokes_base_X{k} = [];
        strokes_base_Y{k} = [];
        continue;
    end
    
    X = pts(:,1);
    Y = pts(:,2);
    
    % 2.1) Remuestreo
    if paso_cm > 0
        [Xr, Yr] = remuestrear_trazo(X, Y, paso_cm);
    else
        Xr = X;
        Yr = Y;
    end
    
    % Asegurar vectores columna
    Xr = Xr(:);
    Yr = Yr(:);
    
    % 2.2) Rotación + offset
    [Xt, Yt] = transformar_trazo(Xr, Yr, angulo_deg, offsetX_cm, offsetY_cm);
    
    strokes_base_X{k} = Xt(:);
    strokes_base_Y{k} = Yt(:);
end

%% 3) Optimizar el orden de los trazos (nearest-neighbor + posible reversa)

% Filtrar trazos vacíos
validIdx = find(~cellfun(@isempty, strokes_base_X));
Nvalid   = numel(validIdx);

if Nvalid == 0
    error('No hay trazos válidos después de remuestreo/transformación.');
end

% Puntos de inicio y fin de cada trazo válido
startPts = zeros(Nvalid, 2);
endPts   = zeros(Nvalid, 2);

for i = 1:Nvalid
    k  = validIdx(i);
    Xk = strokes_base_X{k};
    Yk = strokes_base_Y{k};
    startPts(i,:) = [Xk(1),      Yk(1)];
    endPts(i,:)   = [Xk(end),    Yk(end)];
end

% Heurística: empezamos en el trazo cuyo punto de inicio tenga menor X
[~, idxStart] = min(startPts(:,1));
currentIdxLocal = idxStart;
usedLocal       = false(Nvalid,1);
usedLocal(currentIdxLocal) = true;

orderLocal  = zeros(Nvalid, 1);   % índice en validIdx
dirLocal    = ones(Nvalid, 1);    % +1 = normal, -1 = reverso

orderLocal(1) = currentIdxLocal;
currentPos = endPts(currentIdxLocal, :);

for step = 2:Nvalid
    bestDist = Inf;
    bestJ    = -1;
    bestDir  = +1;
    
    for j = 1:Nvalid
        if usedLocal(j), continue; end
        
        sP = startPts(j,:);
        eP = endPts(j,:);
        
        dStart = hypot(currentPos(1) - sP(1), currentPos(2) - sP(2));
        dEnd   = hypot(currentPos(1) - eP(1), currentPos(2) - eP(2));
        
        if dStart < dEnd
            dCandidate = dStart;
            dirCandidate = +1;  % start -> end
        else
            dCandidate = dEnd;
            dirCandidate = -1;  % end -> start
        end
        
        if dCandidate < bestDist
            bestDist = dCandidate;
            bestJ    = j;
            bestDir  = dirCandidate;
        end
    end
    
    if bestJ < 0
        remaining = find(~usedLocal);
        bestJ = remaining(1);
        bestDir = +1;
    end
    
    usedLocal(bestJ) = true;
    orderLocal(step) = bestJ;
    dirLocal(step)   = bestDir;
    
    j = bestJ;
    if bestDir == +1
        currentPos = endPts(j,:);
    else
        currentPos = startPts(j,:);
    end
end

% Convertir a índices globales y aplicar dirección
finalStrokes_X = cell(Nvalid, 1);
finalStrokes_Y = cell(Nvalid, 1);

for i = 1:Nvalid
    jLocal  = orderLocal(i);
    dirSign = dirLocal(i);
    kGlobal = validIdx(jLocal);
    
    Xk = strokes_base_X{kGlobal};
    Yk = strokes_base_Y{kGlobal};
    
    if dirSign == +1
        finalStrokes_X{i} = Xk(:);
        finalStrokes_Y{i} = Yk(:);
    else
        finalStrokes_X{i} = flipud(Xk(:));
        finalStrokes_Y{i} = flipud(Yk(:));
    end
end

numFinalStrokes = numel(finalStrokes_X);

fprintf('Orden optimizado de trazos. Trazos finales (sin fusionar): %d\n', numFinalStrokes);

%% 4) Guardar a archivo TXT (con renglón en blanco entre trazos)

fid = fopen(nombreArchivoTXT, 'w');
if fid == -1
    error('No se pudo abrir el archivo TXT para escritura: %s', nombreArchivoTXT);
end

for k = 1:numFinalStrokes
    X = finalStrokes_X{k};
    Y = finalStrokes_Y{k};
    if isempty(X)
        continue;
    end
    
    X = X(:);
    Y = Y(:);
    
    for i = 1:numel(X)
        fprintf(fid, '%.4f\t%.4f\n', X(i), Y(i));
    end
    
    % Renglón en blanco entre trazos
    fprintf(fid, '\n');
end

fclose(fid);

fprintf('Archivo TXT generado: %s\n', nombreArchivoTXT);
fprintf('Formato: X_cm<tab>Y_cm, con renglón en blanco entre trazos.\n');

%% 5) Visualización rápida (opcional)

figure; hold on; axis equal;
title('Trayectorias finales (orden optimizado, sin fusionar) en cm');
xlabel('X (cm)'); ylabel('Y (cm)');
for k = 1:numFinalStrokes
    X = finalStrokes_X{k};
    Y = finalStrokes_Y{k};
    if isempty(X), continue; end
    plot(X, Y, '-');
end
grid on;

%% ===== FUNCIONES LOCALES =====

function [Xr, Yr] = remuestrear_trazo(X, Y, paso)
    % Re-muestrea un trazo (X,Y) para que la distancia entre puntos
    % consecutivos sea aprox. "paso" (en cm).
    
    if numel(X) < 2
        Xr = X;
        Yr = Y;
        return;
    end
    
    X = X(:);
    Y = Y(:);
    
    dX = diff(X);
    dY = diff(Y);
    distSeg  = hypot(dX, dY);
    distAcum = [0; cumsum(distSeg)];
    
    longTotal = distAcum(end);
    if longTotal == 0
        Xr = X(1);
        Yr = Y(1);
        return;
    end
    
    numPuntos = max(2, floor(longTotal / paso) + 1);
    distNueva = linspace(0, longTotal, numPuntos);
    
    Xr = interp1(distAcum, X, distNueva, 'linear')';
    Yr = interp1(distAcum, Y, distNueva, 'linear')';
end

function [Xt, Yt] = transformar_trazo(X, Y, ang_deg, offX, offY)
    % Rotación (ang_deg) + traslación (offX, offY) de un trazo (X,Y).
    
    if isempty(X)
        Xt = X;
        Yt = Y;
        return;
    end
    
    X = X(:);
    Y = Y(:);
    
    ang_rad = deg2rad(ang_deg);
    c = cos(ang_rad);
    s = sin(ang_rad);
    
    Xt =  c * X - s * Y;
    Yt =  s * X + c * Y;
    
    Xt = Xt + offX;
    Yt = Yt + offY;
end
