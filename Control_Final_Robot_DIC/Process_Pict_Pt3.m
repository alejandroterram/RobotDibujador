% === 03: Extraer trayectorias (vectores de puntos) desde el esqueleto ===
% Requiere:
%   - 'imagen_lineas.mat' generado por 02_linearizar_imagen.m
%
% Hace:
%   - Toma Iskel (líneas de 1 pixel)
%   - Encuentra componentes conectados
%   - Dentro de cada componente genera trayectorias ordenadas (strokes)
%   - Convierte pixeles -> centímetros usando cmPorPx
%
% Salida:
%   - 'trayectorias.mat' con:
%         strokes_px : cell array, cada celda = [N x 2] = [fila, columna]
%         strokes_cm : cell array, cada celda = [N x 2] = [x_cm, y_cm]
%         cmPorPx, ladoReal_cm
%
% Además muestra una figura con las trayectorias dibujadas.

clear; clc; close all;

%% 0) Parámetros para filtrado de trazos pequeños

minStrokeLength = 5;   % mínimo de puntos en una trayectoria para conservarla

%% 1) Cargar datos de líneas (esqueleto)

datos = load('imagen_lineas.mat');

if ~isfield(datos, 'Iskel')
    error('imagen_lineas.mat no contiene la variable Iskel.');
end

Iskel       = datos.Iskel;
cmPorPx     = datos.cmPorPx;
ladoReal_cm = datos.ladoReal_cm;

[H, W] = size(Iskel);

fprintf('Cargado Iskel de tamaño %d x %d\n', H, W);
fprintf('cmPorPx = %.4f cm/px, ladoReal = %.1f cm\n', cmPorPx, ladoReal_cm);


%% 2) Encontrar componentes conectados en el esqueleto

% Usamos conectividad 8 para seguir líneas diagonalmente también
CC = bwconncomp(Iskel, 8);

fprintf('Componentes conectados encontrados: %d\n', CC.NumObjects);

strokes_px = {};
strokes_cm = {};
strokeCount = 0;

% Para mapear rápidamente pixeles a índices locales, usaremos containers.Map
for c = 1:CC.NumObjects
    
    idxList = CC.PixelIdxList{c};
    numPts  = numel(idxList);
    
    if numPts < minStrokeLength
        % Componente demasiado pequeña, la ignoramos
        continue;
    end
    
    [rowsComp, colsComp] = ind2sub([H, W], idxList);
    
    % Mapa: índice lineal de la imagen completa -> índice local en este componente
    keys   = uint32(idxList(:));
    values = uint32(1:numPts);
    pixMap = containers.Map(keys, values);   % KeyType y ValueType deducidos
    
    % Precalcular grados (número de vecinos en 8-conectividad)
    deg = zeros(numPts,1);
    for k = 1:numPts
        r = rowsComp(k);
        c2 = colsComp(k);
        
        neighCount = 0;
        for dr = -1:1
            for dc = -1:1
                if dr == 0 && dc == 0
                    continue;
                end
                rr = r + dr;
                cc = c2 + dc;
                if rr >= 1 && rr <= H && cc >= 1 && cc <= W
                    lin = sub2ind([H, W], rr, cc);
                    if pixMap.isKey(uint32(lin))
                        neighCount = neighCount + 1;
                    end
                end
            end
        end
        
        deg(k) = neighCount;
    end
    
    visitedLocal = false(numPts, 1);
    
    % Mientras queden pixeles sin visitar, generamos nuevas trayectorias
    while any(~visitedLocal)
        
        % Elegimos un pixel de inicio:
        %  - Preferencia: un pixel extremo (deg == 1)
        %  - Si no hay (ej. bucle cerrado), agarramos cualquiera
        candidates = find(~visitedLocal & (deg == 1));
        if isempty(candidates)
            candidates = find(~visitedLocal);
        end
        startIdxLocal = candidates(1);
        
        % Recorremos este trazo
        pathIdxLocal = [];
        current = startIdxLocal;
        
        while true
            if visitedLocal(current)
                break;
            end
            
            visitedLocal(current) = true;
            pathIdxLocal(end+1,1) = current; %#ok<AGROW>
            
            % Buscamos vecinos NO visitados de current
            r = rowsComp(current);
            c2 = colsComp(current);
            
            neighs = [];
            for dr = -1:1
                for dc = -1:1
                    if dr == 0 && dc == 0
                        continue;
                    end
                    rr = r + dr;
                    cc = c2 + dc;
                    if rr >= 1 && rr <= H && cc >= 1 && cc <= W
                        lin = sub2ind([H, W], rr, cc);
                        if pixMap.isKey(uint32(lin))
                            idxLocal = pixMap(uint32(lin));
                            if ~visitedLocal(idxLocal)
                                neighs(end+1) = idxLocal; %#ok<AGROW>
                            end
                        end
                    end
                end
            end
            
            if isempty(neighs)
                % Llegamos al final de este trazo
                break;
            elseif numel(neighs) == 1
                % Solo un siguiente vecino, seguimos derecho
                current = neighs(1);
            else
                % Varios vecinos sin visitar:
                % elegimos el de menor grado (más "lineal")
                [~, idxMin] = min(deg(neighs));
                current = neighs(idxMin);
            end
        end
        
        % Guardamos este stroke si no es muy pequeño
        if numel(pathIdxLocal) >= minStrokeLength
            strokeCount = strokeCount + 1;
            
            rStroke = rowsComp(pathIdxLocal);
            cStroke = colsComp(pathIdxLocal);
            
            % Puntos en pixeles (fila, columna)
            strokes_px{strokeCount} = [rStroke, cStroke];
            
            % Convertir a coordenadas físicas (cm)
            % Asumimos:
            %   x_cm asociado a columna
            %   y_cm asociado a fila
            x_cm = (cStroke - 1) * cmPorPx;
            y_cm = (rStroke - 1) * cmPorPx;
            
            strokes_cm{strokeCount} = [x_cm, y_cm];
        end
        
    end
end

fprintf('Trayectorias generadas: %d\n', numel(strokes_px));


%% 3) Visualización rápida de las trayectorias

figure;
imshow(Iskel); hold on;
title('Trayectorias extraídas sobre el esqueleto');
for k = 1:numel(strokes_px)
    pts = strokes_px{k};
    plot(pts(:,2), pts(:,1), 'LineWidth', 1);  % columna = x, fila = y
end
set(gca, 'YDir', 'reverse');


%% 4) Guardar resultados

save('trayectorias.mat', ...
     'strokes_px', 'strokes_cm', ...
     'cmPorPx', 'ladoReal_cm', ...
     'minStrokeLength');

fprintf('Trayectorias guardadas en trayectorias.mat\n');
