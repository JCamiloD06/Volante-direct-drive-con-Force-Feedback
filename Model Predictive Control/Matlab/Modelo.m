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

disp('=== MODELO CONTINUO (ESPACIO DE ESTADOS) ===')
sys_c

%% --- FUNCIÓN DE TRANSFERENCIA CONTINUA ---
% Partiendo de la ecuación dinámica del volante:
%
%     J*theta_2punto + b*theta_punto = tau
%
% Aplicando Laplace (condiciones iniciales nulas):
%
%     J*s^2*Theta(s) + b*s*Theta(s) = Tau(s)
%
% Función de transferencia Theta(s)/Tau(s):
%
%     G(s) = 1 / (J*s^2 + b*s)

fprintf('\n')
disp('=== DERIVACIÓN DE LA FUNCIÓN DE TRANSFERENCIA ===')
fprintf('Ecuación dinámica del volante:\n')
fprintf('    J*theta_2punto(t) + b*theta_punto(t) = tau(t)\n\n')
fprintf('Con J = %.4f kg·m^2 y b = %.4f N·m·s/rad:\n\n', J, b)
fprintf('Aplicando la Transformada de Laplace (C.I. = 0):\n')
fprintf('    (%.4f*s^2 + %.4f*s) * Theta(s) = Tau(s)\n\n', J, b)
fprintf('Función de transferencia G(s) = Theta(s) / Tau(s):\n\n')
fprintf('              1\n')
fprintf('    G(s) = -------------\n')
fprintf('           J*s^2 + b*s\n\n')
fprintf('Sustituyendo valores numéricos:\n\n')
fprintf('              1\n')
fprintf('    G(s) = -----------------\n')
fprintf('           %.4f*s^2 + %.4f*s\n\n', J, b)

% Numerador y denominador de la FT (a partir de J y b)
num = 1;
den = [J, b, 0];

sys_tf = tf(num, den, ...
    'InputName',  'Torque (Nm)', ...
    'OutputName', 'Angulo (rad)');

disp('=== MODELO CONTINUO (FUNCIÓN DE TRANSFERENCIA) ===')
sys_tf

% También obtenida directamente a partir del modelo en espacio de estados
% (debe coincidir con sys_tf, es solo una verificación cruzada)
sys_tf_check = tf(sys_c);
disp('=== VERIFICACIÓN: FT obtenida desde ss(sys_c) ===')
sys_tf_check

% --- Información detallada de la FT (polos, ceros, ganancia, DC gain) ---
[zeros_tf, poles_tf, k_tf] = zpkdata(sys_tf, 'v');

fprintf('\n')
disp('=== CARACTERÍSTICAS DE LA FUNCIÓN DE TRANSFERENCIA ===')
fprintf('Numerador:            [%s]\n', num2str(num))
fprintf('Denominador:          [%s]\n', num2str(den))
fprintf('Ganancia (k):         %.6f\n', k_tf)
if isempty(zeros_tf)
    fprintf('Ceros:                Ninguno\n')
else
    fprintf('Ceros:                %s\n', num2str(zeros_tf.'))
end
fprintf('Polos:                %s\n', num2str(poles_tf.'))
fprintf('Orden del sistema:    %d\n', length(poles_tf))
fprintf('Ganancia DC:          %.6f  (infinita si hay polo en s=0 -> tipo integrador)\n', dcgain(sys_tf))
fprintf('\n')
fprintf('Interpretación de los polos:\n')
fprintf('  - Polo en s = 0        -> integrador puro (el ángulo integra la velocidad)\n')
fprintf('  - Polo en s = -b/J = %.4f rad/s -> dinámica de primer orden en velocidad\n', -b/J)
fprintf('  - Constante de tiempo asociada: tau = J/b = %.4f s\n', J/b)

%% --- DISCRETIZACIÓN ---
Ts = 0.05;
sys_d = c2d(sys_c, Ts, 'zoh');

disp('=== MODELO DISCRETO (Ts = 0.01s) — ESPACIO DE ESTADOS ===')
sys_d

%% --- FUNCIÓN DE TRANSFERENCIA DISCRETA ---
sys_tf_d = c2d(sys_tf, Ts, 'zoh');

disp('=== MODELO DISCRETO — FUNCIÓN DE TRANSFERENCIA ===')
sys_tf_d

[zeros_tf_d, poles_tf_d, k_tf_d] = zpkdata(sys_tf_d, 'v');

fprintf('\n')
disp('=== CARACTERÍSTICAS DE LA FUNCIÓN DE TRANSFERENCIA DISCRETA ===')
fprintf('Periodo de muestreo (Ts): %.4f s\n', Ts)
fprintf('Ganancia (k):              %.6f\n', k_tf_d)
if isempty(zeros_tf_d)
    fprintf('Ceros:                     Ninguno\n')
else
    fprintf('Ceros:                     %s\n', num2str(zeros_tf_d.'))
end
fprintf('Polos:                     %s\n', num2str(poles_tf_d.'))
fprintf('\n')
fprintf('Interpretación de los polos discretos:\n')
fprintf('  - Polo en z = 1                 -> integrador puro discreto (arrastrado del polo continuo en s=0)\n')
fprintf('  - Polo en z = exp(-b/J * Ts) = %.6f -> dinámica de velocidad discretizada (ZOH)\n', exp(-b/J*Ts))

%% --- VERIFICACIÓN: Respuesta al escalón ---
figure;
step(sys_c, 5);
title('Respuesta del volante a torque escalón de 1 Nm');
xlabel('Tiempo (s)');
ylabel('Ángulo \theta (rad)');
grid on;