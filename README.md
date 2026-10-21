# Practica-Robotica-20-10-2026
% Cinemática Inversa y Animación de Robot Planar de 2 GDL
clear; clc; close all;

%% 1. Leer x_F, y_F, l1, l2[cite: 1]
xF = input('Ingrese x_F: ');
yF = input('Ingrese y_F: ');
l1 = input('Ingrese l1: ');
l2 = input('Ingrese l2: ');

r = sqrt(xF^2 + yF^2);

% Validación de alcanzabilidad del objetivo
if r > (l1 + l2) || r < abs(l1 - l2)
    error('El punto objetivo está fuera del alcance del robot.');
end

%% 2. Preguntar opción de codo[cite: 1]
disp(' ');
disp('Seleccione la configuración del codo:');
disp('1: Codo arriba');
disp('2: Codo abajo');
opcion = input('Opción (1 o 2): ');

%% 3, 4 y 6. Cálculos de ángulos (sigma, beta, gamma)[cite: 1]
sigma = atan2d(yF, xF); %[cite: 1]
beta  = acosd((l2^2 - l1^2 - r^2) / (-2 * l1 * r)); %[cite: 1]
gamma = acosd((r^2 - l1^2 - l2^2) / (-2 * l1 * l2)); %[cite: 1]

%% 5, 7 y 8. Definir theta1 y theta2 según la configuración del codo[cite: 1]
if opcion == 1 % Codo arriba
    theta1_final = beta + sigma;         % theta1 = beta + sigma[cite: 1]
    theta2_final = -(180 - gamma);       % Giro negativo[cite: 1]
else % Codo abajo
    theta1_final = sigma - beta;         % theta1 = sigma - beta
    theta2_final = (180 - gamma);        % Giro positivo[cite: 1]
end

%% Animación y trazado de trayectoria
pasos = 60; % Cantidad de cuadros de la animación
th1_trayectoria = linspace(0, theta1_final, pasos);
th2_trayectoria = linspace(0, theta2_final, pasos);

% Configuración del escenario gráfico
figure('Name', 'Simulación de Cinemática Inversa', 'NumberTitle', 'off');
grid on; hold on; axis equal;
limite = (l1 + l2) * 1.2;
xlim([-limite, limite]);
ylim([-limite, limite]);
xlabel('Eje X'); ylabel('Eje Y');
title('Movimiento del Robot de 2 GDL');

% Marcador del punto objetivo
plot(xF, yF, 'rx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Objetivo');

% Objetos gráficos para el brazo y la traza
h_brazo = plot([0, 0, 0], [0, 0, 0], '-o', 'LineWidth', 3, ...
    'MarkerFaceColor', 'k', 'MarkerSize', 6, 'DisplayName', 'Brazo');
h_traza = plot(0, 0, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Trayectoria');

legend('Location', 'northeast');

x_efector = zeros(1, pasos);
y_efector = zeros(1, pasos);

% Bucle para generar el movimiento
for k = 1:pasos
    q1 = th1_trayectoria(k);
    q2 = th2_trayectoria(k);

    % Cinemática Directa para obtener las posiciones de los eslabones
    x1 = l1 * cosd(q1);
    y1 = l1 * sind(q1);

    x2 = x1 + l2 * cosd(q1 + q2);
    y2 = y1 + l2 * sind(q1 + q2);

    % Almacenar posición del extremo para trazar la línea
    x_efector(k) = x2;
    y_efector(k) = y2;

    % Actualizar datos en pantalla
    set(h_brazo, 'XData', [0, x1, x2], 'YData', [0, y1, y2]);
    set(h_traza, 'XData', x_efector(1:k), 'YData', y_efector(1:k));

    drawnow;
    pause(0.03); % Velocidad de refresco de la animación
end

% Resultados impresos en consola
fprintf('\n---------------- RESULTADOS ----------------\n');
fprintf('Theta 1 objetivo: %.2f°\n', theta1_final);
fprintf('Theta 2 objetivo: %.2f°\n', theta2_final);
fprintf('--------------------------------------------\n');
