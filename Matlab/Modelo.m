%% ================================================
%  PROYECTO: MPC VOLANTE HÁPTICO
%  Paso 2: Modelo del sistema en espacio de estados
%% ================================================
clear; clc;

%% --- PARÁMETROS FÍSICOS (estimados) ---
J = 0.08;       % Inercia rotacional total    [kg·m²]
b = 0.05;       % Fricción viscosa total      [N·m·s/rad]

% Cuando tengas el volante real, reemplaza estos valores
% con los identificados experimentalmente

%% --- ESPACIO DE ESTADOS CONTINUO ---
% Estado:   x = [theta; theta_punto]
% Entrada:  u = tau (torque del motor)
% Salida:   y = theta (ángulo medido por encoder)

A = [0,      1;
     0,   -b/J];

B = [0;
     1/J];

C = [1, 0];
D = 0;

sys_c = ss(A, B, C, D, ...
    'StateName',  {'Angulo (rad)', 'Velocidad (rad/s)'}, ...
    'InputName',  {'Torque (Nm)'}, ...
    'OutputName', {'Angulo (rad)'});

disp('=== MODELO CONTINUO ===')
sys_c

%% --- DISCRETIZACIÓN ---
% Ts = 10ms → 100 Hz
% El MPC correrá a esta frecuencia en el STM32
Ts = 0.01;
sys_d = c2d(sys_c, Ts, 'zoh');

disp('=== MODELO DISCRETO (Ts = 0.01s) ===')
sys_d

%% --- VERIFICACIÓN: Respuesta al escalón ---
figure;
step(sys_c, 5);
title('Respuesta del volante a torque escalón de 1 Nm');
xlabel('Tiempo (s)');
ylabel('Ángulo \theta (rad)');
grid on;