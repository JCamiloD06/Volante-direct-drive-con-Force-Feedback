%% =======================================================================
%  mpc_ambos_modelos.m
%
%  Desarrollo matemático completo de LOS DOS controladores MPC del
%  proyecto MPC Monza:
%
%    PARTE 1-2 — MPC de DIRECCIÓN (volante)
%       Derivación del modelo físico del actuador, discretización,
%       formulación batch (Phi, Gamma) — es una QP genuina, lineal — y
%       resolución + simulación de lazo cerrado.
%
%    PARTE 3-6 — MPC LONGITUDINAL (acelerador/freno)
%       Derivación del modelo de throttle (primer orden K/tau) Y del
%       modelo de freno (desaceleración CONSTANTE — la estructura nueva,
%       distinta de throttle), horizonte de predicción no uniforme, y
%       resolución + simulación del sistema conmutado.
%
%    PARTE 7 — Perfil de velocidad por curvatura (contexto: es la
%       referencia que alimenta al MPC longitudinal, no un tercer MPC)
%
%  Corre el script completo con F5. No requiere archivos externos.
%
%  Requiere: MATLAB base + (opcional) Optimization Toolbox (fmincon). Si
%  no lo tienes, el script avisa y salta las partes que necesitan resolver
%  un problema de optimización real (pero las derivaciones matemáticas y
%  las gráficas de referencia sí se muestran de todas formas).
% =========================================================================

clear; clc; close all;

has_fmincon = exist('fmincon','file') == 2 || exist('fmincon','file') == 6;
if ~has_fmincon
    warning(['No se detectó el Optimization Toolbox (fmincon). Las ' ...
             'simulaciones de lazo cerrado de este script no van a poder ' ...
             'optimizar de verdad. Instala el toolbox, o corre esto en ' ...
             'MATLAB Online (lo incluye por defecto), para ver las ' ...
             'simulaciones completas.']);
end


%% =========================================================================
%  PARTE 1 — MPC DE DIRECCIÓN: MODELO FÍSICO Y DISCRETIZACIÓN
% =========================================================================
%
%  A diferencia del control longitudinal (que controla la física interna
%  de AC, una caja negra), la dirección SÍ controla un actuador físico
%  real con dinámica propia y conocida: el motor del volante.
%
%  Segunda ley de Newton rotacional sobre el eje del volante:
%
%       J * theta_ddot = tau - b * theta_dot                        (1)
%
%  donde:
%    J        = inercia del volante+motor (kg*m^2)
%    b        = fricción viscosa (amortiguamiento, N*m*s/rad)
%    tau      = torque aplicado por el motor (N*m) — esta es la entrada u
%    theta    = ángulo del volante (rad) — este es el estado que queremos
%               llevar a la referencia

fprintf('=== PARTE 1: Modelo del actuador de dirección ===\n\n');

J = 0.02;   % kg*m^2 (inercia)
b = 0.10;   % N*m*s/rad (fricción viscosa)
Ts_dir = 0.05;   % paso de actuación (50 ms)

fprintf('Modelo continuo: J*theta_ddot = tau - b*theta_dot\n');
fprintf('  J=%.3f kg*m^2,  b=%.3f N*m*s/rad\n\n', J, b);

% ---- Espacio de estados continuo ----
% x = [theta; theta_dot],  u = tau
%   theta_dot     = theta_dot                     (trivial)
%   theta_ddot    = -(b/J)*theta_dot + (1/J)*tau
%
%   x_dot = Ac*x + Bc*u
%   Ac = [0 1; 0 -b/J]      Bc = [0; 1/J]
Ac = [0 1; 0 -b/J];
Bc = [0; 1/J];
fprintf('Espacio de estados continuo:\n  Ac = [0 1; 0 %.3f]\n  Bc = [0; %.2f]\n\n', -b/J, 1/J);

% ---- Discretización (Euler hacia adelante) ----
%   x[k+1] = x[k] + Ts*(Ac*x[k] + Bc*u[k])
%          = (I + Ts*Ac)*x[k] + Ts*Bc*u[k]
%   =>  Ad = I + Ts*Ac       Bd = Ts*Bc
Ad = eye(2) + Ts_dir*Ac;
Bd = Ts_dir*Bc;

fprintf('Discretización Euler (Ts=%.3fs):\n', Ts_dir);
fprintf('  Ad = [1, %.4f; 0, %.4f]\n', Ad(1,2), Ad(2,2));
fprintf('  Bd = [%.5f; %.4f]\n\n', Bd(1), Bd(2));

C = [1 0];  % la salida que nos importa es theta (no theta_dot directamente)


%% =========================================================================
%  PARTE 2 — MPC DE DIRECCIÓN: FORMULACIÓN BATCH (QP genuina) Y SOLUCIÓN
% =========================================================================
%
%  A diferencia del MPC longitudinal (donde el freno introduce un sistema
%  CONMUTADO, no lineal), la dirección SÍ es un problema de control
%  cuadrático puro: el modelo es lineal, sin conmutación entre modos.
%
%  Igual que en la Parte 2 del script de solo-longitudinal, se condensa la
%  predicción completa en forma matricial:
%
%       Theta = Phi*x0 + Gamma*U                                     (2)
%
%  pero ahora con un estado de 2 componentes [theta; theta_dot] y una
%  matriz C que extrae solo theta (lo único que aparece en el costo):
%
%       Phi(k,:)   = C * Ad^k                    (fila 1x2, k=1..N)
%       Gamma(i,j) = C * Ad^(i-j) * Bd            (escalar, j<=i)
%
%  Costo (idéntico en estructura al de Python, MPCSteeringController):
%
%     J = sum_{k=1}^{N} w_k*(theta_k - theta_ref_k)^2
%       + R*sum u_k^2 + Rd*sum (u_k - u_{k-1})^2
%
%     w_k = Q para k<N,  w_N = Qf  (peso extra al último paso del horizonte)
%
%  Restricciones:
%     |u_k|              <= tau_max
%     |u_k - u_{k-1}|    <= rate_max
%     |theta_k|          <= theta_max

fprintf('=== PARTE 2: MPC de dirección — formulación batch ===\n\n');

N_dir = 10;             % horizonte (10 x 0.05s = 0.5s, igual que en Python)
Q_dir  = 10.41;
Qf_dir = 10.41;
R_dir  = 0.0;
Rd_dir = 0.288;
tau_max   = 2.0;   % Nm
rate_max  = 1.0;   % Nm por paso
theta_max = 1.2;   % rad

Phi_dir = zeros(N_dir, 2);
Gamma_dir = zeros(N_dir, N_dir);
Ad_pows = cell(N_dir+1, 1);
Ad_pows{1} = eye(2);
for k = 1:N_dir
    Ad_pows{k+1} = Ad_pows{k} * Ad;   % Ad^k acumulado
end
for i = 1:N_dir
    Phi_dir(i,:) = C * Ad_pows{i+1};          % C*Ad^i
    for j = 1:i
        Gamma_dir(i,j) = C * Ad_pows{i-j+1} * Bd;  % C*Ad^(i-j)*Bd
    end
end

fprintf('Phi (dirección): %dx2   |   Gamma (dirección): %dx%d\n', N_dir, N_dir, N_dir);
fprintf('(Verificación: theta_1 = Phi(1,:)*x0 + Gamma(1,1)*u0, debe ser igual a\n');
fprintf(' un paso de la recursión x1 = Ad*x0 + Bd*u0 extrayendo la componente theta)\n\n');

% ---- Simulación de lazo cerrado con referencia tipo "chicana" ----
% Referencia sintética: una S de ángulos (como pasar por una chicana),
% para ejercitar el MPC con cambios de signo, no solo un escalón.
T_total_dir = 4.0;
n_steps_dir = round(T_total_dir/Ts_dir);
t_vec_dir = (0:n_steps_dir-1)*Ts_dir;
theta_ref_profile = 0.6*sin(2*pi*0.5*t_vec_dir) .* (t_vec_dir < 3.0);

x0 = [0; 0];       % estado inicial [theta; theta_dot]
u_prev_dir = 0.0;
u_warm_dir = zeros(N_dir,1);

theta_log = zeros(1, n_steps_dir);
tau_log   = zeros(1, n_steps_dir);

if has_fmincon
    opts_dir = optimoptions('fmincon','Display','none','Algorithm','sqp','MaxIterations',40);
end

w_dir = Q_dir*ones(N_dir,1);
w_dir(end) = Qf_dir;   % peso extra al último paso del horizonte

for k = 1:n_steps_dir
    % referencia sobre el horizonte: mantiene el último valor conocido si
    % nos quedamos sin datos hacia el final de la simulación
    idxs = min(k:(k+N_dir-1), n_steps_dir);
    theta_ref_k = theta_ref_profile(idxs)';

    cost_fun = @(U) costo_direccion(U, x0, theta_ref_k, Phi_dir, Gamma_dir, ...
                                     w_dir, R_dir, Rd_dir, u_prev_dir);
    nonlcon_fun = @(U) restricciones_direccion(U, x0, Phi_dir, Gamma_dir, ...
                                                theta_max, rate_max, u_prev_dir);

    lb = -tau_max*ones(N_dir,1);
    ub =  tau_max*ones(N_dir,1);

    if has_fmincon
        U_opt = fmincon(cost_fun, u_warm_dir, [],[],[],[], lb, ub, nonlcon_fun, opts_dir);
    else
        U_opt = u_warm_dir;
    end

    tau_apply = U_opt(1);
    x0 = Ad*x0 + Bd*tau_apply;   % aplica un paso real de la planta

    theta_log(k) = x0(1);
    tau_log(k) = tau_apply;

    u_warm_dir = [U_opt(2:end); U_opt(end)];
    u_prev_dir = tau_apply;
end

fprintf('Simulación de dirección completa: %d pasos de %.3fs\n\n', n_steps_dir, Ts_dir);

figure('Name','MPC Dirección - Lazo cerrado');
subplot(2,1,1);
plot(t_vec_dir, theta_ref_profile, 'k--', 'LineWidth', 1.2); hold on;
plot(t_vec_dir, theta_log, 'b-', 'LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('\theta (rad)');
legend('\theta_{ref}','\theta_{real}','Location','best');
title('Seguimiento de ángulo de volante (MPC dirección)');
grid on;

subplot(2,1,2);
plot(t_vec_dir, tau_log, 'r-', 'LineWidth', 1.2);
yline(tau_max,'k:'); yline(-tau_max,'k:');
xlabel('Tiempo (s)'); ylabel('\tau (Nm)');
title('Torque aplicado (líneas punteadas = límite \pm\tau_{max})');
grid on;


%% =========================================================================
%  PARTE 3 — MPC LONGITUDINAL: MODELO DE THROTTLE (primer orden K/tau)
% =========================================================================
%
%  Igual que en el script anterior de solo-longitudinal: el acelerador no
%  tiene un actuador físico propio que modelar desde primeros principios
%  (AC es una caja negra), así que se identifica empíricamente como un
%  sistema de un polo:
%
%       tau_th * dv/dt + v = K_th * u          (u en [0,1] para throttle)
%
%  Identificado con step_test_logger.py + identify_model.py sobre datos
%  reales: tau_th=2.81s (consistente entre amplitudes de prueba, la
%  ganancia K_th es más incierta por saturación de RPM en la marcha usada
%  — ver README_acelerador_freno.md del proyecto para el detalle completo).

fprintf('=== PARTE 3: Modelo de throttle ===\n\n');

K_th   = 250.0;    % km/h -- ajusta según la velocidad máxima real de tu auto
tau_th = 2.81;      % s    -- identificado empíricamente, confiable

fprintf('Modelo continuo (throttle): tau_th*dv/dt + v = K_th*u\n');
fprintf('  K_th = %.1f km/h,  tau_th = %.2f s\n\n', K_th, tau_th);

Ts_long = 0.2;   % paso de PREDICCIÓN interno (no de actuación, ver Parte 5)
Ad_th = 1 - Ts_long/tau_th;
Bd_th = Ts_long/tau_th * K_th;

fprintf('Discretización Euler (ts_pred=%.2fs): Ad_th=%.4f, Bd_th=%.4f\n\n', Ts_long, Ad_th, Bd_th);


%% =========================================================================
%  PARTE 4 — MPC LONGITUDINAL: MODELO DE FRENO (desaceleración constante)
% =========================================================================
%
%  ESTE ES EL CAMBIO IMPORTANTE respecto a versiones anteriores: el freno
%  NO se modela como un sistema de primer orden K/tau — se modela como
%  desaceleración CONSTANTE, y la razón es física, no de conveniencia:
%
%  El acelerador satura de forma SUAVE porque la resistencia que se opone
%  al motor (arrastre aerodinámico) crece con v^2 — a más velocidad, más
%  le cuesta seguir acelerando, de ahí la forma exponencial que se aplana.
%
%  El freno, en cambio, aplica una fuerza de fricción que es
%  aproximadamente CONSTANTE en el rango de operación normal (no depende
%  fuertemente de v):
%
%       F_freno ≈ mu * m * g   (fricción neumático-pista, casi constante)
%
%  Por Newton:  m * dv/dt = -F_freno  =>  dv/dt = -a_brake_max   (constante)
%
%  Esto es un modelo de RAMPA (integrador puro con entrada constante), no
%  un sistema de un polo — y es justo lo que se confirmó al identificar
%  con datos reales: un ajuste LINEAL a la velocidad de frenado dio
%  R²≈0.999, contra R²≈0.83-0.94 forzando una exponencial (ver
%  README_acelerador_freno.md, sección 6, para el detalle completo del
%  hallazgo con datos reales).
%
%  Discretizando directamente (ya es lineal en el tiempo, no hace falta
%  Euler de una ODE):
%
%       v[k+1] = max( 0, v[k] - Ts * a_brake_max * |u| )             (3)
%
%  El max(0, ...) es OBLIGATORIO: el auto se detiene y se queda detenido,
%  no puede "seguir frenando" hacia velocidades negativas — es la
%  diferencia física clave frente al modelo de throttle, que si no se
%  controla puede crecer sin límite matemático (aunque en la práctica el
%  MPC nunca le pediría eso).

fprintf('=== PARTE 4: Modelo de freno (desaceleración constante) ===\n\n');

a_brake_max_ms2 = 10.25;                  % identificado empíricamente (m/s^2)
a_brake_max = a_brake_max_ms2 * 3.6;      % convertido a km/h/s (mismas unidades que v)

fprintf('Modelo: dv/dt = -a_brake_max*|u|   (NO es tau*dv/dt+v=K*u)\n');
fprintf('  a_brake_max = %.2f m/s² = %.1f km/h/s\n\n', a_brake_max_ms2, a_brake_max);

fprintf('Discretización directa (ts_pred=%.2fs):\n', Ts_long);
fprintf('  v[k+1] = max(0, v[k] - ts*a_brake_max*|u|)\n\n');


%% =========================================================================
%  PARTE 5 — HORIZONTE DE PREDICCIÓN NO UNIFORME (multi-rate)
% =========================================================================
%
%  Las dos plantas (dirección y longitudinal) tienen dinámicas de
%  velocidad MUY distinta:
%
%       tau_direccion  = J/b = %.2f s
%       tau_throttle   ≈ 2.81 s        (casi 15x más lenta)
%
%  Si el MPC longitudinal predijera con el mismo Ts=0.05s que usa
%  dirección, un horizonte de N=15 pasos solo cubriría 0.75s — menos de
%  un tercio de tau_throttle, así que el MPC sería miope respecto a su
%  propia física (nunca "vería" su dinámica asentarse).
%
%  Solución (ya implementada arriba, Ts_long=0.2s en vez de 0.05s): el
%  MPC longitudinal sigue APLICANDO comandos cada 0.05s (misma frecuencia
%  de actuación que dirección — reacciona igual de rápido a cambios),
%  pero predice internamente con pasos de 0.2s, así que N=15 pasos cubren
%  3.0s de horizonte — sí alcanza a cubrir tau_throttle. Es la técnica de
%  "move blocking" / rejilla de predicción no uniforme, estándar en MPC
%  de sistemas con dinámicas de múltiple escala temporal.

fprintf('=== PARTE 5: Horizonte no uniforme ===\n\n');
fprintf('  tau_direccion (J/b) = %.3f s\n', J/b);
fprintf('  tau_throttle        = %.2f s  (%.0fx más lenta)\n', tau_th, tau_th/(J/b));
fprintf('  Horizonte dirección:   N=%d x Ts=%.2fs = %.2fs (%.1fx tau_direccion)\n', ...
    N_dir, Ts_dir, N_dir*Ts_dir, (N_dir*Ts_dir)/(J/b));
fprintf('  Horizonte longitudinal: N=15 x ts_pred=%.2fs = %.1fs (%.1fx tau_throttle)\n\n', ...
    Ts_long, 15*Ts_long, (15*Ts_long)/tau_th);


%% =========================================================================
%  PARTE 6 — MPC LONGITUDINAL: RESOLVER EL SISTEMA CONMUTADO (fmincon)
% =========================================================================
%
%  A diferencia de dirección (QP pura, Parte 2), el longitudinal es un
%  SISTEMA CONMUTADO: qué ecuación aplica (throttle o freno) depende del
%  SIGNO de cada u_i en la secuencia, así que Gamma ya no es una matriz
%  fija — depende de la propia solución que se busca. Por eso se resuelve
%  con un solver de programación no lineal (fmincon), no con quadprog.

fprintf('=== PARTE 6: Simulación de lazo cerrado longitudinal ===\n\n');

N_long = 15;
Q_long = 10.0; R_long = 0.5; Rrate_long = 2.0;
u_min = -1.0; u_max = 1.0;
u_prev_long = 0.0;

T_total_long = 15;
n_steps_long = round(T_total_long/Ts_dir);   % se ACTÚA a Ts_dir=0.05s
t_vec_long = (0:n_steps_long-1)*Ts_dir;

v_ref_profile = zeros(1,n_steps_long);
for k = 1:n_steps_long
    tk = t_vec_long(k);
    if tk < 8
        v_ref_profile(k) = 100;
    elseif tk < 10
        v_ref_profile(k) = 0;      % frenada completa hasta detenerse
    else
        v_ref_profile(k) = 120;
    end
end

v_log = zeros(1,n_steps_long);
u_log = zeros(1,n_steps_long);
v = 0;

if has_fmincon
    opts_long = optimoptions('fmincon','Display','none','Algorithm','sqp','MaxIterations',30);
end

u_warm_long = zeros(N_long,1);

for k = 1:n_steps_long
    v_ref_k = v_ref_profile(k) * ones(N_long,1);

    cost_fun = @(U) costo_longitudinal(U, v, v_ref_k, Q_long, R_long, Rrate_long, ...
                                        u_prev_long, Ad_th, Bd_th, a_brake_max, Ts_long);

    lb = u_min*ones(N_long,1);
    ub = u_max*ones(N_long,1);

    if has_fmincon
        U_opt = fmincon(cost_fun, u_warm_long, [],[],[],[], lb, ub, [], opts_long);
    else
        U_opt = u_warm_long;
    end

    u_apply = U_opt(1);
    u_log(k) = u_apply;

    if u_apply >= 0
        v = v + (Ts_dir/tau_th)*(-v + K_th*u_apply);
    else
        v = max(0, v - Ts_dir*a_brake_max*(-u_apply));
    end
    v_log(k) = v;

    u_warm_long = [U_opt(2:end); U_opt(end)];
    u_prev_long = u_apply;
end

fprintf('Simulación longitudinal completa: %d pasos de %.3fs (%.0fs totales)\n\n', ...
    n_steps_long, Ts_dir, T_total_long);

figure('Name','MPC Longitudinal - Lazo cerrado (freno = desaceleración constante)');
subplot(2,1,1);
plot(t_vec_long, v_ref_profile, 'k--', 'LineWidth', 1.2); hold on;
plot(t_vec_long, v_log, 'b-', 'LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('Velocidad (km/h)');
legend('v_{ref}','v_{real}','Location','best');
title('Seguimiento de velocidad — nótese la frenada en LÍNEA RECTA hacia 0');
grid on;

subplot(2,1,2);
plot(t_vec_long, u_log, 'r-', 'LineWidth', 1.2);
yline(0,'k:');
xlabel('Tiempo (s)'); ylabel('u (comando)');
title('Comando de control (+ = acelerador, - = freno)');
ylim([-1.1 1.1]); grid on;


%% =========================================================================
%  PARTE 7 — PERFIL DE VELOCIDAD POR CURVATURA (contexto — no es un MPC,
%             es la referencia que alimenta al MPC longitudinal)
% =========================================================================
%
%  v_max(s) = sqrt(ay_max/|kappa(s)|), seguido de un backward-pass que
%  limita la velocidad según cuánto se puede frenar (Torricelli) desde el
%  punto siguiente. Se incluye aquí solo como contexto de dónde sale
%  v_ref en el sistema real — el detalle completo de esta derivación ya
%  se cubrió en mpc_longitudinal_matematica.m (Parte 4 de ese archivo).

fprintf('=== PARTE 7: Perfil de velocidad (contexto) ===\n\n');

n_pts = 300;
s = linspace(0, 600, n_pts);
ds = mean(diff(s));
kappa = zeros(1,n_pts);
kappa(s >= 200 & s <= 280) = 0.02;

ay_max = 6.5*0.75; ax_eff = 5.5*0.75; v_cap = 83.3;
kappa_safe = max(abs(kappa), 1e-5);
v_curva = min(sqrt(ay_max ./ kappa_safe), v_cap);

v_max = v_curva;
for pasada = 1:3
    for i = n_pts:-1:1
        nxt = mod(i, n_pts) + 1;
        v_permitida = sqrt(max(0, v_max(nxt)^2 + 2*ax_eff*ds));
        if v_permitida < v_max(i)
            v_max(i) = v_permitida;
        end
    end
end

figure('Name','Perfil de velocidad (contexto)');
plot(s, v_curva*3.6, 'b--', 'LineWidth', 1); hold on;
plot(s, v_max*3.6, 'r-', 'LineWidth', 1.8);
xlabel('Distancia s (m)'); ylabel('Velocidad (km/h)');
legend('límite por curvatura', 'tras backward pass de frenado', 'Location','best');
title('v_{ref} para el MPC longitudinal (generado por SpeedProfileGenerator)');
grid on;

fprintf('=== FIN DEL SCRIPT ===\n');


%% =========================================================================
%  FUNCIONES AUXILIARES (deben ir al final del script en MATLAB)
% =========================================================================

function J = costo_direccion(U, x0, theta_ref, Phi, Gamma, w, R, Rd, u_prev)
    theta_vec = Phi*x0 + Gamma*U;
    cost_tracking = sum(w .* (theta_vec - theta_ref).^2);
    cost_effort = R * sum(U.^2);
    U_ext = [u_prev; U];
    cost_rate = Rd * sum(diff(U_ext).^2);
    J = cost_tracking + cost_effort + cost_rate;
end

function [c, ceq] = restricciones_direccion(U, x0, Phi, Gamma, theta_max, rate_max, u_prev)
    theta_vec = Phi*x0 + Gamma*U;
    N = length(U);
    Ddiff = eye(N) - diag(ones(N-1,1), -1);
    corr = zeros(N,1); corr(1) = u_prev;
    du = Ddiff*U - corr;   % du(i) = U(i)-U(i-1), con du(1)=U(1)-u_prev

    c = [ theta_vec - theta_max;      % theta_k <= theta_max
         -theta_vec - theta_max;      % theta_k >= -theta_max
          du - rate_max;              % du <= rate_max
         -du - rate_max ];            % du >= -rate_max
    ceq = [];
end

function J = costo_longitudinal(U, v0, v_ref, Q, R, Rrate, u_prev, ...
                                  Ad_th, Bd_th, a_brake_max, ts)
    N = length(U);
    v = zeros(N,1);
    vk = v0;
    for i = 1:N
        if U(i) >= 0
            vk = Ad_th*vk + Bd_th*U(i);        % throttle: primer orden
        else
            vk = max(0, vk - ts*a_brake_max*(-U(i)));  % freno: rampa con clip en 0
        end
        v(i) = vk;
    end

    costo_tracking = sum( Q * (v - v_ref).^2 );
    costo_effort = sum( R * U.^2 );
    U_ext = [u_prev; U];
    costo_rate = sum( Rrate * diff(U_ext).^2 );

    J = costo_tracking + costo_effort + costo_rate;
end
