% === 02: Obtener líneas (bordes + esqueleto) para sketch del robot ===
% Supone que ya corriste ANTES el script 01 de:
%   - Escala de grises
%   - Redimensionamiento (lado más largo = 15 cm)
%   - (Opcionalmente filtrado)
%
% Entrada:
%   archivo 'imagen_preprocesada.mat' generado por el script 01
%
% Salida:
%   archivo 'imagen_lineas.mat' con:
%       Iskel      -> esqueleto (líneas de 1 pixel)
%       Ibw_clean  -> bordes limpios
%       edges      -> bordes Canny crudos
%       cmPorPx    -> cm por pixel (para pasar a coordenadas reales)
%       ladoReal_cm
%
% Solo muestra:
%   1) Imagen de trabajo (la que viene del script 1)
%   2) Bordes limpios
%   3) Esqueleto final

clear; clc; close all;

%% 0) Parámetros de calibración para las LÍNEAS

sigmaSuavizado = 1.75;   % Suavizado previo a Canny (0.5–2 aprox)
cannyLow       = 0.10;  % Umbral bajo Canny (más bajo = más bordes)
cannyHigh      = 0.25;  % Umbral alto Canny (debe ser > cannyLow)
areaMin        = 20;    % Área mínima en pixeles para conservar un trazo

% Tips:
% - Muy pocos bordes  -> bajar cannyLow y cannyHigh (p.ej. 0.05, 0.15)
% - Mucho ruido       -> subir cannyLow y cannyHigh, o subir areaMin
% - Demasiada textura -> subir sigmaSuavizado o subir areaMin


%% 1) Cargar la imagen preprocesada del script 01

datos = load('imagen_preprocesada.mat');
fprintf('Cargado: imagen_preprocesada.mat\n');

% Intentamos detectar el nombre de la imagen de trabajo:
if isfield(datos, 'Ifilt')
    Iwork = datos.Ifilt;     % imagen filtrada del script 1
elseif isfield(datos, 'Ires')
    Iwork = datos.Ires;      % imagen reescalada en gris
elseif isfield(datos, 'Igray')
    Iwork = datos.Igray;     % imagen en gris
else
    error('No encontré Ifilt, Ires ni Igray en imagen_preprocesada.mat');
end

% Recuperar escala física si existe
if isfield(datos, 'cmPorPx')
    cmPorPx = datos.cmPorPx;
elseif isfield(datos, 'ladoReal_cm')
    % Si no está cmPorPx, lo calculamos por si acaso
    ladoReal_cm = datos.ladoReal_cm;
    [r_tmp, c_tmp] = size(Iwork);
    ladoMax_px = max(r_tmp, c_tmp);
    cmPorPx = ladoReal_cm / ladoMax_px;
else
    % En el peor de los casos, inventamos 1 cm/px (luego lo ajustas)
    warning('No encontré cmPorPx ni ladoReal_cm. Asignando cmPorPx = 1.');
    cmPorPx = 1;
    ladoReal_cm = 1;
end

if isfield(datos, 'ladoReal_cm')
    ladoReal_cm = datos.ladoReal_cm;
else
    [r_tmp, c_tmp] = size(Iwork);
    ladoReal_cm = cmPorPx * max(r_tmp, c_tmp);
end

figure;
imshow(Iwork);
title('Imagen de trabajo (gris + reescalada desde script 01)');


%% 2) Suavizado previo (para quitar textura y ruido fino)

Ismooth = imgaussfilt(Iwork, sigmaSuavizado);


%% 3) Bordes con Canny

edges = edge(Ismooth, 'Canny', [cannyLow cannyHigh]);


%% 4) Limpieza de ruido (quitar trazos muy pequeños)

Ibw_clean = bwareaopen(edges, areaMin);

figure;
imshow(Ibw_clean);
title(sprintf('Bordes limpios (areaMin = %d px)', areaMin));


%% 5) Esqueleto (adelgazar a 1 pixel de grosor)

Iskel = bwmorph(Ibw_clean, 'thin', Inf);

% Opcional: quitar ramitas cortas
% Iskel = bwmorph(Iskel, 'spur', 5);

figure;
imshow(Iskel);
title('Esqueleto final (líneas de 1 pixel)');


%% 6) Guardar resultado para el siguiente paso (trayectorias)

save('imagen_lineas.mat', ...
     'Iwork', ...       % imagen en gris reescalada
     'edges', ...       % bordes Canny crudos
     'Ibw_clean', ...   % bordes limpios
     'Iskel', ...       % esqueleto final
     'cmPorPx', ...     % cm por pixel
     'ladoReal_cm', ... % lado más largo en cm
     'sigmaSuavizado', ...
     'cannyLow', 'cannyHigh', ...
     'areaMin');

fprintf('\nProcesamiento de LÍNEAS completo.\n');
fprintf('Guardado en imagen_lineas.mat\n');
fprintf('cmPorPx = %.4f cm/px, ladoReal = %.1f cm\n', cmPorPx, ladoReal_cm);
