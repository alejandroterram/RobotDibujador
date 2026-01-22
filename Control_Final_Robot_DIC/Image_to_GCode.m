%% IMAGE -> XY COORDS for 2DOF plotter
% - Lee una imagen JPEG
% - La reescala a un área de 15x15 cm
% - Convierte a blanco y negro con máximo contraste
% - Genera puntos (X,Y) en cm siguiendo un barrido tipo raster
% - Aplica un offset +10 cm en Y (para el brazo)
% - Guarda coords15.txt con dos columnas: X Y
% - Plotea todas las etapas y la trayectoria final
%
% Luego usarás Script 2:
%   inputFile  = 'coords15.txt';
%   ... para hacer IK y generar el G-code.
%
% NOTA: Aquí NO se usa M3/M5 ni se genera G-code.

clear; clc; close all;

%% ---------- USER PARAMETERS ---------------------------------------------
imageFile   = 'input_image.jpg';   % <-- pon aquí tu JPEG
outputCoords = 'coords15.txt';     % <-- archivo de salida (X Y en cm)

% Área física de dibujo (cm)
width_cm    = 15;   % eje X (cm)
height_cm   = 15;   % eje Y "útil" (cm)

% Offset en Y para que el brazo se pueda desdoblar
yOffset     = 10;   % cm (Y final irá de 10 a 25 aprox.)

% Resolución de la grilla (más px/cm = más puntos)
px_per_cm   = 8;    % p.ej. 8 px/cm -> 120x120 puntos aprox.
Ny          = round(height_cm * px_per_cm);  % filas
Nx          = round(width_cm  * px_per_cm);  % columnas

%% ---------- 1) LEER IMAGEN ---------------------------------------------
Irgb = imread(imageFile);

figure('Name','Image processing pipeline','NumberTitle','off');
subplot(2,3,1);
imshow(Irgb);
title('1) Original RGB');

%% ---------- 2) ESCALA DE GRISES ----------------------------------------
if size(Irgb,3) == 3
    Igray = rgb2gray(Irgb);
else
    Igray = Irgb;
end
subplot(2,3,2);
imshow(Igray);
title('2) Grayscale');

%% ---------- 3) CONTRASTE MÁXIMO (IMADJUST) -----------------------------
Iadj = imadjust(Igray);    % estira contraste a [0,1]
subplot(2,3,3);
imshow(Iadj);
title('3) Contrast adjusted');

%% ---------- 4) BINARIZAR (BLANCO/NEGRO) --------------------------------
% Primero binarizamos, luego invertimos para que "tinta" = 1
BW0 = imbinarize(Iadj);    % umbral de Otsu
BW  = ~BW0;                % zonas oscuras = 1 (tinta)
subplot(2,3,4);
imshow(BW);
title('4) Binary (ink = white pixels)');

%% ---------- 5) REDIMENSIONAR A LA GRILLA DE DIBUJO ---------------------
BW_resized = imresize(BW, [Ny Nx], 'nearest'); % conserva bordes nítidos
subplot(2,3,5);
imshow(BW_resized);
title(sprintf('5) Resized to %dx%d pixels', Ny, Nx));

%% ---------- 6) GENERAR COORDENADAS XY (RASTER SCAN) --------------------
% Mapeo de coordenadas físicas:
%   x: 0 .. width_cm
%   y: 0 .. height_cm, pero luego sumamos yOffset
% Nota: fila 1 de la imagen está arriba, así que invertimos Y para
% que y=0 esté abajo antes del offset.

dx = width_cm  / (Nx - 1);
dy = height_cm / (Ny - 1);

coordsX = [];
coordsY = [];

for i = 1:Ny
    % Máscara de la fila i (1 = tinta)
    rowMask = BW_resized(i,:);

    % Y en coordenadas de dibujo (antes del offset)
    y_base  = (Ny - i) * dy;   % fila 1 -> Y ~ height_cm
    y_robot = y_base + yOffset;

    % Barrido serpenteante para minimizar saltos
    if mod(i,2) == 1
        % izquierda -> derecha
        idxOrder    = 1:Nx;
        rowSerpMask = rowMask;
    else
        % derecha -> izquierda
        idxOrder    = Nx:-1:1;
        rowSerpMask = rowMask(idxOrder);
    end

    for jj = 1:Nx
        if rowSerpMask(jj) == 1
            c = idxOrder(jj);          % índice de columna original
            x_robot = (c - 1) * dx;    % X en cm

            coordsX(end+1,1) = x_robot; %#ok<SAGROW>
            coordsY(end+1,1) = y_robot; %#ok<SAGROW>
        end
    end
end

%% ---------- 7) GUARDAR coords15.txt (X Y en cm) ------------------------
fid = fopen(outputCoords, 'w');
if fid == -1
    error('No se pudo abrir el archivo %s para escritura.', outputCoords);
end

for k = 1:numel(coordsX)
    fprintf(fid, '%.4f %.4f\n', coordsX(k), coordsY(k));
end

fclose(fid);
fprintf('Listo. Se guardaron %d puntos en %s\n', numel(coordsX), outputCoords);

%% ---------- 8) PLOT DE LA TRAYECTORIA FINAL ----------------------------
subplot(2,3,6);
plot(coordsX, coordsY, '.-');
axis equal;
xlim([0 width_cm]);
ylim([yOffset, yOffset + height_cm]);
set(gca, 'YDir', 'normal');   % (0,0) abajo-izquierda
xlabel('X [cm]');
ylabel('Y [cm]');
title('6) Final drawing trajectory (XY with offset)');
grid on;
