%% =======================================================================
%  CALCULOS DE ENGRANAJES - VOLANTE FFB
%  Metodo: Ecuacion de Lewis para esfuerzo de flexion en dientes de engranaje
% =======================================================================
clear; clc;

%% --- 1. Datos del motor (motor de hoverboard) ---
Torque_motor = 10;          % [Nm]
Potencia     = 250;         % [W] = [Nm/s]
V_kmh        = 15;          % [km/h] velocidad lineal de referencia
V            = V_kmh/3.6;   % [m/s] -> 4.1667 m/s

%% --- 2. Datos geometricos de los engranajes ---
dE = 66;      % [mm] diametro engranaje conducido (grande)
zE = 44;      % [dientes] engranaje conducido
dP = 33;      % [mm] diametro pinon (chico, motriz)
zP = 22;      % [dientes] pinon
m  = 1.5;     % [mm] modulo
b  = 10;      % [mm] ancho de cara (face width)
relacion = dE/dP;                 % relacion de transmision (2:1)
dc = (dE + dP)/2;                 % [mm] distancia entre centros aprox.

fprintf('Relacion de transmision: %.2f : 1\n', relacion);
fprintf('Distancia entre centros (dc): %.2f mm\n\n', dc);

%% --- 3. Factores de servicio y velocidad ---
Cs = 1.8;                 % Factor de servicio: 8-10 h/dia, choque pesado (heavy shock)
Cv = 4.58/(4.58 + V);     % Factor de velocidad (engranaje cuidadosamente cortado)

%% --- 4. Fuerza tangencial y fuerza maxima de diseno ---
Ft   = Potencia / V;          % [N] Fuerza tangencial = Potencia/Velocidad
Fmax = Cs * Ft / Cv;          % [N] Fuerza maxima de diseno

fprintf('Fuerza tangencial (Ft):  %.5f N\n', Ft);
fprintf('Factor Cv:               %.6f\n', Cv);
fprintf('Fuerza maxima (Fmax):    %.6f N\n\n', Fmax);

%% --- 5. Factor de forma de Lewis (Y) ---
% Se evalua sobre el pinon (zP = 22 dientes), diente mas critico a flexion
Y = pi * (0.154 - 0.912/zP);
fprintf('Factor de forma de Lewis (Y): %.15f\n\n', Y);

%% --- 6. Resistencia a la flexion de materiales candidatos [MPa = N/mm^2] ---
materiales = {'Policarbonato (PC)', 'ABS', 'PLA'};
sigma_d    = [111, 70.5, 103];   % [N/mm^2] esfuerzo admisible a flexion

%% --- 7. Calculo de Fbeam (capacidad de carga del diente) por material ---
% Ecuacion de Lewis: Fbeam = b * m * sigma_d * Y
Fbeam = b .* m .* sigma_d .* Y;

fprintf('%-22s %-15s %-15s %-10s\n', 'Material', 'sigma_d [MPa]', 'Fbeam [N]', 'Cumple?');
fprintf('---------------------------------------------------------------\n');
for i = 1:length(materiales)
    if Fbeam(i) >= Fmax
        estado = 'SI (Fbeam >= Fmax)';
    else
        estado = 'NO (Fbeam < Fmax)';
    end
    fprintf('%-22s %-15.2f %-15.5f %-10s\n', materiales{i}, sigma_d(i), Fbeam(i), estado);
end

%% --- 8. Resumen / criterio de diseno ---
fprintf('\nCriterio de diseno: Fbeam >= Fmax (%.5f N)\n', Fmax);
fprintf('Los tres materiales cumplen el criterio estructural minimo.\n');
fprintf('El Policarbonato (PC) ofrece el mayor margen de seguridad,\n');
fprintf('seguido de PLA y luego ABS.\n');

%% --- 9. Grafica comparativa (opcional) ---
figure;
bar(categorical(materiales), Fbeam);
hold on;
yline(Fmax, 'r--', 'LineWidth', 2, 'Label', 'Fmax (fuerza requerida)');
ylabel('Fbeam [N]');
title('Capacidad de carga del diente (Fbeam) por material vs Fmax');
grid on;
