% === Preprocesar imagen para sketch de robot ===
% Pasos:
% 1) Cargar imagen (JPG, PNG, etc.)
% 2) Convertir a escala de grises
% 3) Redimensionar para que el LADO MÁS LARGO represente 15 cm reales
% 4) Mejorar contraste
% 5) Filtrar ruido
% 6) Guardar info de escala física (cm/px)

clear; clc; close all;

%% 0) Parámetros del mundo real
ladoReal_cm = 15;      % Lado MÁS LARGO de la imagen en centímetros (real)
pxPorCm_deseado = 65;  % Resolución: pixeles por centímetro (ajústalo a tu gusto)

% Pixeles que debería tener el lado más largo
ladoMax_dest_px = ladoReal_cm * pxPorCm_deseado;

%% 1) Cargar imagen
nombre = ['' ...
    '1.jpg'];   % <- cambia esto por tu archivo
I = imread(nombre);

fprintf('Imagen cargada: %s\n', nombre);

%% 2) Convertir a escala de grises
if size(I,3) == 3
    Igray = rgb2gray(I);
else
    Igray = I;
end

Igray = im2double(Igray);

figure;
imshow(Igray);
title('Paso 1: Imagen en escala de grises');

%% 3) Redimensionar para que el lado más largo represente 15 cm

[rows, cols] = size(Igray);
ladoMax_orig_px = max(rows, cols);

% Factor de escala para que el lado más largo tenga ladoMax_dest_px pixeles
scale = ladoMax_dest_px / ladoMax_orig_px;

% Si NO quieres agrandar imágenes pequeñas, descomenta esta línea:
% scale = min(scale, 1);

Ires = imresize(Igray, scale);

[rows_res, cols_res] = size(Ires);
ladoMax_res_px = max(rows_res, cols_res);

% Cálculo real de pixeles por cm y cm por pixel (por si hay redondeos)
pxPorCm_real = ladoMax_res_px / ladoReal_cm;   % px/cm realmente obtenido
cmPorPx      = ladoReal_cm / ladoMax_res_px;   % cm por cada pixel

figure;
imshow(Ires);
title(sprintf('Paso 2: Redimensionada (lado max = %.1f cm, ~%.2f px/cm)', ...
    ladoReal_cm, pxPorCm_real));

fprintf('Lado más largo real: %d px -> %.1f cm (%.3f cm/px)\n', ...
    ladoMax_res_px, ladoReal_cm, cmPorPx);

%% 4) Mejorar contraste

% === High-Boost Filtering para un contraste MUY fuerte ===
% alpha controla qué tan agresivo quieres el contraste
alpha = .1;     % prueba 1.3, 1.6, 2.0, 2.5 (más grande = contraste más duro)

blurred = imgaussfilt(Ires, 50);          % suavizado para separar tonos
highBoost = Ires + (Ires - blurred) * alpha;

% === Ecualización adaptativa para reflejar más detalle ===
Icontrast = adapthisteq(highBoost, 'ClipLimit', 0.02);

figure;
imshow(Icontrast);
title('Contraste ULTRA mejorado (High-Boost + CLAHE)');


%% 5) Filtrar ruido (elige uno: mediana o gaussiano)

% Opción A: Filtro mediano (bueno para ruido tipo sal y pimienta)
Ifilt = medfilt2(Icontrast, [3 3]);

% Opción B: Filtro gaussiano (si prefieres suavizado más "suave")
% Ifilt = imgaussfilt(Icontrast, 1);  % sigma = 1

figure;
imshow(Ifilt);
title('Paso 4: Imagen filtrada (menos ruido)');

%% 6) Guardar resultado para siguientes pasos (binarización, trayectorias, etc.)

save('imagen_preprocesada.mat', ...
     'Ifilt', ...          % imagen lista para binarizar
     'scale', ...          % escala aplicada sobre la original
     'ladoReal_cm', ...    % 15 cm (lado más largo en el mundo real)
     'pxPorCm_real', ...   % resolución real px/cm
     'cmPorPx');           % factor para pasar de pixeles a cm

fprintf('Preprocesado completo.\n');
fprintf('Guardado en imagen_preprocesada.mat (cmPorPx = %.4f cm/px)\n', cmPorPx);
