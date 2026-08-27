%% ================================================
%  VALIDACIÓN COMPLETA DEL SISTEMA MPC
%  Ejecutar después de simular en Simulink
%% ================================================

%% EXTRAER DATOS DE LA SIMULACIÓN
X     = out.X_pos.Data;
Y     = out.Y_pos.Data;
t     = out.tout;
Ts    = 0.05;

%% ================================================
%  MÉTRICA 1 — ERROR DE SEGUIMIENTO DEL VOLANTE
%% ================================================
% Necesitas tener guardado θ_ref y θ_real en To Workspace
% Si no los tienes agrega dos bloques To Workspace en Simulink:
%   theta_ref  → salida del Gain pi/180 × 180/pi (en grados)
%   theta_real → salida del bloque Volante × 180/pi (en grados)

 %Si ya los tienes:
 theta_ref_data  = out.theta_ref.Data;
 theta_real_data = out.theta_real.Data;
 error_seguimiento = theta_ref_data - theta_real_data;

 RMSE = sqrt(mean(error_seguimiento.^2));
 MAE  = mean(abs(error_seguimiento));
 MAX  = max(abs(error_seguimiento));
 
 fprintf('\n=== MÉTRICA 1: ERROR DE SEGUIMIENTO ===\n')
 fprintf('RMSE:  %.4f grados\n', RMSE)
 fprintf('MAE:   %.4f grados\n', MAE)
 fprintf('Error máximo: %.4f grados\n', MAX)
 
% ¿Qué debes ver?
%   RMSE < 5°    → seguimiento excelente
%   RMSE < 10°   → seguimiento aceptable
%   RMSE > 15°   → hay que resintonizar

%% ================================================
%  MÉTRICA 2 — TRAYECTORIA XY DEL VEHÍCULO
%% ================================================
fprintf('\n=== MÉTRICA 2: TRAYECTORIA XY ===\n')
fprintf('Distancia total recorrida: %.2f m\n', ...
    sum(sqrt(diff(X).^2 + diff(Y).^2)))
fprintf('Desviación lateral máxima: %.4f m\n', max(abs(Y)))
fprintf('Desviación lateral mínima: %.4f m\n', min(Y))
fprintf('Rango lateral total: %.4f m\n', max(Y) - min(Y))
fprintf('Tiempo de simulación: %.1f s\n', t(end))

% ¿Qué debes ver?
%   Distancia total > 0        → el carro se mueve ✓
%   Desviación lateral > 0     → el carro vira ✓
%   Rango lateral proporcional → a la referencia dada

%% ================================================
%  MÉTRICA 3 — SUAVIDAD DE LA TRAYECTORIA
%% ================================================
% Curvatura de la trayectoria
dx = gradient(X);
dy = gradient(Y);
ddx = gradient(dx);
ddy = gradient(dy);
curvatura = abs(dx.*ddy - dy.*ddx) ./ (dx.^2 + dy.^2).^1.5;
curvatura(isnan(curvatura)) = 0;

fprintf('\n=== MÉTRICA 3: SUAVIDAD DE TRAYECTORIA ===\n')
fprintf('Curvatura media: %.6f 1/m\n', mean(curvatura))
fprintf('Curvatura máxima: %.6f 1/m\n', max(curvatura))

% ¿Qué debes ver?
%   Curvatura suave y continua → el MPC genera movimientos suaves ✓
%   Picos de curvatura → cambios bruscos (indeseable)

%% ================================================
%  MÉTRICA 4 — VELOCIDAD DEL VEHÍCULO
%% ================================================
Vx_aprox = gradient(X) / Ts;
Vy_aprox = gradient(Y) / Ts;
V_total  = sqrt(Vx_aprox.^2 + Vy_aprox.^2);

fprintf('\n=== MÉTRICA 4: VELOCIDAD ===\n')
fprintf('Velocidad media: %.4f m/s\n', mean(V_total))
fprintf('Velocidad máxima: %.4f m/s\n', max(V_total))
fprintf('Velocidad mínima: %.4f m/s\n', min(V_total(round(end/2):end)))

% ¿Qué debes ver?
%   Velocidad ≈ 11 m/s constante → el modelo es correcto ✓
%   Variaciones grandes → problema en el modelo

%% ================================================
%  GRÁFICAS DE VALIDACIÓN
%% ================================================
figure('Position', [50, 50, 1400, 900]);

% Versión sin flechas — más simple
subplot(3,2,[1,2])
plot(X, Y, 'b-', 'LineWidth', 2.5);
hold on;
plot(X(1),   Y(1),   'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot(X(end), Y(end), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');

% Puntos de dirección cada ciertos pasos
step = round(length(X)/15);
scatter(X(1:step:end), Y(1:step:end), 30, 'k', 'filled');

title('Trayectoria XY del Vehículo — Control MPC', 'FontSize', 13);
xlabel('X (m)'); ylabel('Y (m)');
legend('Trayectoria MPC', 'Inicio', 'Fin', 'Location', 'best');
grid on;

% GRÁFICA 2 — Desviación lateral Y
subplot(3,2,3)
plot(t, Y, 'r-', 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1);
yline(max(Y),  'g--', sprintf('Máx: %.3fm', max(Y)));
yline(min(Y),  'b--', sprintf('Mín: %.3fm', min(Y)));
title('Desviación Lateral Y vs Tiempo', 'FontSize', 12);
xlabel('Tiempo (s)'); ylabel('Y (m)');
grid on;

% GRÁFICA 3 — Posición X vs Tiempo
subplot(3,2,4)
plot(t, X, 'b-', 'LineWidth', 2);
hold on;
plot(t, 11*t, 'k--', 'LineWidth', 1.5);   % referencia v=5m/s
title('Posición X vs Tiempo', 'FontSize', 12);
xlabel('Tiempo (s)'); ylabel('X (m)');
legend('X real', 'Referencia v=5m/s', 'Location', 'best');
grid on;

% GRÁFICA 4 — Curvatura
subplot(3,2,5)
plot(t, curvatura, 'm-', 'LineWidth', 1.5);
title('Curvatura de la Trayectoria', 'FontSize', 12);
xlabel('Tiempo (s)'); ylabel('κ (1/m)');
grid on;

% GRÁFICA 5 — Velocidad resultante
subplot(3,2,6)
plot(t, V_total, 'g-', 'LineWidth', 1.5);
yline(11, 'k--', 'v = 11 m/s', 'LineWidth', 1.5);
title('Velocidad Resultante del Vehículo', 'FontSize', 12);
xlabel('Tiempo (s)'); ylabel('V (m/s)');
grid on;

sgtitle('VALIDACIÓN SISTEMA MPC — Volante Háptico de Bajo Costo', ...
        'FontSize', 14, 'FontWeight', 'bold');

saveas(gcf, 'Validacion_MPC_Completa.png', 'png');
disp('Figura de validación guardada ✓')