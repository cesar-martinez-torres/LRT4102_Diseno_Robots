%% diff_drive_sim_2025b.m
% Simulador 2D de robot diferencial (MATLAB 2025b)
% - Cinemática diferencial (unicycle)
% - Control go-to-goal (v, w)
% - Conversión a ruedas
% - Opción de dinámica de ruedas 1er orden
% - Animación y gráficas
clear; clc; close all;

%% ===================== Parámetros del robot ============================
p.r = 0.05;      % [m] radio de rueda
p.L = 0.30;      % [m] distancia entre ruedas (track width)

% Límites físicos (ajusta según tu robot)
p.v_max = 0.8;           % [m/s]
p.w_max = 2.5;           % [rad/s]
p.wheel_w_max = 20;      % [rad/s] límite de velocidad angular rueda

% Opción: dinámica simple de ruedas (más realista)
useWheelDynamics = true;
p.tau_wheel = 0.15;      % [s] constante de tiempo (1er orden). Si 0 -> ideal

%% ===================== Control go-to-goal ==============================
% Ganancias (punto de partida)
k.krho   = 1.2;     % >0
k.kalpha = 3.5;     % >0
k.kbeta  = -1.2;    % <0

% Objetivo (pose)
goal.x = 2.0;    % [m]
goal.y = 1.5;    % [m]
goal.th = deg2rad(45);  % [rad]

% Umbrales para detener
tol_rho = 0.03;       % [m]
tol_th  = deg2rad(2); % [rad]

%% ===================== Simulación ======================================
dt = 0.02;         % [s] (50 Hz)
T  = 20;           % [s]
N  = round(T/dt) + 1;
t  = (0:N-1)*dt;
animStep = 1;      % Actualiza dibujo cada N pasos
animSpeed = 0.6;   % 1.0 = tiempo real, <1 mas lento, >1 mas rapido

% Estado: [x y th]
x = zeros(N,1); y = zeros(N,1); th = zeros(N,1);

% Estado inicial
x(1)  = 0.0;
y(1)  = 0.0;
th(1) = deg2rad(0);

% Variables
v_cmd   = zeros(N,1);
w_cmd   = zeros(N,1);
wR_ref  = zeros(N,1);
wL_ref  = zeros(N,1);
wR      = zeros(N,1);  % velocidad real rueda (si hay dinámica)
wL      = zeros(N,1);
v_real  = zeros(N,1);
w_real  = zeros(N,1);
rho_hist = zeros(N,1);
alpha_hist = zeros(N,1);
beta_hist = zeros(N,1);

% Inicializa ruedas reales
wR(1) = 0; wL(1) = 0;

%% ===================== Figura / animación ==============================
fig = figure('Color','w'); ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlabel(ax,'X [m]'); ylabel(ax,'Y [m]');
title(ax,'Robot diferencial - simulación 2D');

% Dibuja objetivo
plot(ax, goal.x, goal.y, 'kx', 'MarkerSize', 10, 'LineWidth', 2);
text(goal.x+0.05, goal.y+0.05, 'Goal', 'Parent', ax);

trajPlot = plot(ax, x(1), y(1), 'b-', 'LineWidth', 1.5);
robotBody = plot(ax, nan, nan, 'k-', 'LineWidth', 2);
robotWheel1 = patch(ax, nan, nan, [0.75 0.75 0.75], 'EdgeColor', 'k', 'LineWidth', 1.2);
robotWheel2 = patch(ax, nan, nan, [0.75 0.75 0.75], 'EdgeColor', 'k', 'LineWidth', 1.2);
robotCenter = plot(ax, nan, nan, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
robotHeading = quiver(ax, x(1), y(1), cos(th(1)), sin(th(1)), 0.2, 'LineWidth',2);
[bx0, by0, w1x0, w1y0, w2x0, w2y0] = robotFootprint(x(1), y(1), th(1), p.L);
set(robotBody, 'XData', bx0, 'YData', by0);
set(robotWheel1, 'XData', w1x0, 'YData', w1y0);
set(robotWheel2, 'XData', w2x0, 'YData', w2y0);
set(robotCenter, 'XData', x(1), 'YData', y(1));

% Ventana de vista
xlim(ax, [-0.5, max(goal.x,2.5)+0.5]);
ylim(ax, [-0.5, max(goal.y,2.0)+0.5]);

%% ===================== Loop ============================================
stopped = false;

for i = 1:N-1
    % 1) Errores a meta
    dx = goal.x - x(i);
    dy = goal.y - y(i);

    rho   = hypot(dx, dy);
    alpha = wrapToPiLocal(atan2(dy, dx) - th(i));
    beta  = wrapToPiLocal(goal.th - th(i) - alpha);

    rho_hist(i) = rho; alpha_hist(i) = alpha; beta_hist(i) = beta;

    % 2) Ley de control (go-to-goal)
    if rho < tol_rho
        % Cerca del objetivo: sólo alinea orientación
        v = 0.0;
        w = sat(k.kalpha * wrapToPiLocal(goal.th - th(i)), p.w_max);
        if abs(wrapToPiLocal(goal.th - th(i))) < tol_th
            v = 0; w = 0; stopped = true;
        end
    else
        v = k.krho   * rho;
        w = k.kalpha * alpha + k.kbeta * beta;
        v = sat(v, p.v_max);
        w = sat(w, p.w_max);
    end

    v_cmd(i) = v; w_cmd(i) = w;

    % 3) (v,w) -> ruedas (referencia)
    wRr = (v + (p.L/2)*w) / p.r;
    wLr = (v - (p.L/2)*w) / p.r;

    % Saturación por rueda
    wRr = sat(wRr, p.wheel_w_max);
    wLr = sat(wLr, p.wheel_w_max);

    wR_ref(i) = wRr;
    wL_ref(i) = wLr;

    % 4) Dinámica de ruedas (opcional)
    if useWheelDynamics && p.tau_wheel > 0
        % Primer orden: w_dot = (w_ref - w)/tau
        wR(i+1) = wR(i) + (dt/p.tau_wheel) * (wRr - wR(i));
        wL(i+1) = wL(i) + (dt/p.tau_wheel) * (wLr - wL(i));
    else
        wR(i+1) = wRr;
        wL(i+1) = wLr;
    end

    % 5) Ruedas reales -> (v,w) reales
    v_now = (p.r/2) * (wR(i+1) + wL(i+1));
    w_now = (p.r/p.L) * (wR(i+1) - wL(i+1));

    v_real(i) = v_now;
    w_real(i) = w_now;

    % 6) Integración de la cinemática
    [x(i+1), y(i+1), th(i+1)] = integrateUnicycle(x(i), y(i), th(i), v_now, w_now, dt);

    % 7) Animación ligera
    if mod(i, animStep) == 0
        set(trajPlot, 'XData', x(1:i+1), 'YData', y(1:i+1));
        [bx, by, w1x, w1y, w2x, w2y] = robotFootprint(x(i+1), y(i+1), th(i+1), p.L);
        set(robotBody, 'XData', bx, 'YData', by);
        set(robotWheel1, 'XData', w1x, 'YData', w1y);
        set(robotWheel2, 'XData', w2x, 'YData', w2y);
        set(robotCenter, 'XData', x(i+1), 'YData', y(i+1));
        set(robotHeading, 'XData', x(i+1), 'YData', y(i+1), ...
            'UData', 0.25*cos(th(i+1)), 'VData', 0.25*sin(th(i+1)));
        drawnow;
        pause((dt * animStep) / max(animSpeed, 1e-3));
    end

    if stopped
        % rellena historial restante
        v_cmd(i+1:end)  = 0; w_cmd(i+1:end)  = 0;
        wR_ref(i+1:end) = wR_ref(i); wL_ref(i+1:end) = wL_ref(i);
        wR(i+1:end)     = wR(i+1);   wL(i+1:end)     = wL(i+1);
        break;
    end
end

% último punto de errores
dx = goal.x - x(end); dy = goal.y - y(end);
rho_hist(end)   = hypot(dx,dy);
alpha_hist(end) = wrapToPiLocal(atan2(dy, dx) - th(end));
beta_hist(end)  = wrapToPiLocal(goal.th - th(end) - alpha_hist(end));

%% ===================== Gráficas ========================================
figure('Color','w');
subplot(2,2,1); plot(x,y,'b-', 'LineWidth',1.5); hold on; grid on; axis equal;
plot(goal.x, goal.y,'kx','MarkerSize',10,'LineWidth',2);
xlabel('X [m]'); ylabel('Y [m]'); title('Trayectoria');

subplot(2,2,2); plot(t, v_cmd,'LineWidth',1.2); hold on; plot(t, v_real,'LineWidth',1.2); grid on;
xlabel('t [s]'); ylabel('v [m/s]'); title('Velocidad lineal'); legend('v cmd','v real');

subplot(2,2,3); plot(t, w_cmd,'LineWidth',1.2); hold on; plot(t, w_real,'LineWidth',1.2); grid on;
xlabel('t [s]'); ylabel('\omega [rad/s]'); title('Velocidad angular'); legend('\omega cmd','\omega real');

subplot(2,2,4); plot(t, wR,'LineWidth',1.2); hold on; plot(t, wL,'LineWidth',1.2); grid on;
xlabel('t [s]'); ylabel('\omega_{wheel} [rad/s]'); title('Velocidades de ruedas');
legend('\omega_R','\omega_L');

figure('Color','w');
subplot(3,1,1); plot(t, rho_hist,'LineWidth',1.2); grid on; ylabel('\rho [m]'); title('Errores');
subplot(3,1,2); plot(t, alpha_hist,'LineWidth',1.2); grid on; ylabel('\alpha [rad]');
subplot(3,1,3); plot(t, beta_hist,'LineWidth',1.2); grid on; ylabel('\beta [rad]'); xlabel('t [s]');

disp('Simulación terminada.');
fprintf('Pose final: x=%.3f, y=%.3f, th=%.2f deg\n', x(end), y(end), rad2deg(th(end)));

%% ===================== Funciones locales ===============================
function a = wrapToPiLocal(a)
% Envuelve ángulo a (-pi, pi]
    a = mod(a + pi, 2*pi) - pi;
end

function y = sat(x, lim)
% Saturación simétrica
    y = min(max(x, -lim), lim);
end

function [xn, yn, thn] = integrateUnicycle(x, y, th, v, w, dt)
% Integración más estable que Euler para el unicycle:
% - Si w ~ 0 -> recta
% - Si w != 0 -> arco exacto
    if abs(w) < 1e-6
        xn  = x  + v*cos(th)*dt;
        yn  = y  + v*sin(th)*dt;
        thn = th;
    else
        thn = th + w*dt;
        xn  = x + (v/w)*(sin(thn) - sin(th));
        yn  = y - (v/w)*(cos(thn) - cos(th));
    end
    thn = mod(thn + pi, 2*pi) - pi;
end

function [bx, by, w1x, w1y, w2x, w2y] = robotFootprint(x, y, th, L)
% Dibuja un "chasis" simple (línea entre ruedas) y un pequeño frente
    bodyLen = 1.40 * L;
    bodyWid = 1.05 * L;
    cornerR = 0.22 * L;
    nArc = 10;

    [bxLocal, byLocal] = roundedRectangleLocal(bodyLen, bodyWid, cornerR, nArc);

    wheelLen = 0.36 * L;
    wheelWid = 0.14 * L;
    wheelAng = 0;
    wheel1Center = [0.00 * L;  0.24 * L];
    wheel2Center = [0.00 * L; -0.24 * L];

    wheel1Local = orientedRectLocal(wheel1Center, wheelLen, wheelWid, wheelAng);
    wheel2Local = orientedRectLocal(wheel2Center, wheelLen, wheelWid, wheelAng);

    R = [cos(th) -sin(th); sin(th) cos(th)];
    bodyWorld  = R * [bxLocal; byLocal] + [x; y];
    wheel1World = R * wheel1Local + [x; y];
    wheel2World = R * wheel2Local + [x; y];

    bx = bodyWorld(1, :);
    by = bodyWorld(2, :);
    w1x = wheel1World(1, :);
    w1y = wheel1World(2, :);
    w2x = wheel2World(1, :);
    w2y = wheel2World(2, :);
end

function pts = orientedRectLocal(center, len, wid, ang)
% Rectangulo con orientacion ang (en el marco local del robot)
    base = [...
        -len/2,  len/2,  len/2, -len/2; ...
        -wid/2, -wid/2,  wid/2,  wid/2];
    R = [cos(ang) -sin(ang); sin(ang) cos(ang)];
    pts = R * base + center;
end

function [xv, yv] = roundedRectangleLocal(len, wid, r, nArc)
% Retorna el contorno cerrado de un rectangulo redondeado en marco local
    r = min([r, len/2 - 1e-6, wid/2 - 1e-6]);
    cx = len/2 - r;
    cy = wid/2 - r;

    a1 = linspace(-pi/2, 0, nArc);
    a2 = linspace(0, pi/2, nArc);
    a3 = linspace(pi/2, pi, nArc);
    a4 = linspace(pi, 3*pi/2, nArc);

    p1 = [ cx + r*cos(a1); -cy + r*sin(a1)];
    p2 = [ cx + r*cos(a2);  cy + r*sin(a2)];
    p3 = [-cx + r*cos(a3);  cy + r*sin(a3)];
    p4 = [-cx + r*cos(a4); -cy + r*sin(a4)];

    pts = [p1, p2, p3, p4];
    xv = [pts(1, :), pts(1, 1)];
    yv = [pts(2, :), pts(2, 1)];
end
