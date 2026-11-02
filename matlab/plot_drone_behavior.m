function plot_drone_behavior(logDir, saveFigs)
%PLOT_DRONE_BEHAVIOR  Plot the behaviour of the simulated PX4/Gazebo Iris drone.
%
%   plot_drone_behavior('~/drone_logs')        plot logs from telemetry_logger.py
%   plot_drone_behavior('~/drone_logs', true)  ...and save PNGs next to the logs
%   plot_drone_behavior()                      run on built-in demo data
%
%   Expected files in logDir (written by tools/telemetry_logger.py):
%     gps.csv  attitude.csv  odometry.csv  battery.csv  status.csv
%   Only gps.csv is strictly required; other plots are skipped if missing.
%
%   Figures: 1) 3-D trajectory  2) ground track  3) altitude & speed
%            4) attitude (roll/pitch/yaw) & body rates  5) battery
%            6) flight-mode / arming timeline
%   No toolboxes required (quaternion conversion is done locally).

if nargin < 1 || isempty(logDir), logDir = ''; end
if nargin < 2, saveFigs = false; end

D = loadLogs(logDir);
if isempty(D.gps)
    error('No usable GPS data found in "%s".', logDir);
end

% ---- common time base (PX4 timestamps share one clock) -------------------
t0 = inf;
for f = {'gps','attitude','odometry','battery','status'}
    T = D.(f{1});
    if ~isempty(T), t0 = min(t0, min(T.px4_timestamp_us)); end
end
tsec = @(T) double(T.px4_timestamp_us - t0) * 1e-6;

% ---- GPS -> local ENU (metres) relative to first valid fix ---------------
G = D.gps;
valid = G.fix_type >= 3;
if ~any(valid), valid = true(height(G),1); end   % sim may report odd fix_type
G = G(valid,:);
tg  = tsec(G);
lat = double(G.lat_e7) * 1e-7;
lon = double(G.lon_e7) * 1e-7;
alt = double(G.alt_mm) * 1e-3;
R = 6378137;                                     % WGS-84 equatorial radius
E = deg2rad(lon - lon(1)) * R * cosd(lat(1));
N = deg2rad(lat - lat(1)) * R;
U = alt - alt(1);
spd = hypot(G.vel_n, G.vel_e);                   % horizontal speed
if all(isnan(spd)), spd = [0; hypot(diff(E), diff(N)) ./ diff(tg)]; end

% ---- summary -------------------------------------------------------------
fprintf('\n=== Flight summary ===\n');
fprintf('Duration          : %.1f s\n', tg(end)-tg(1));
fprintf('Max altitude gain : %.2f m\n', max(U));
fprintf('Max horiz. speed  : %.2f m/s\n', max(spd));
fprintf('Path length (3-D) : %.2f m\n', sum(sqrt(diff(E).^2+diff(N).^2+diff(U).^2)));
fprintf('Farthest from home: %.2f m\n\n', max(hypot(E,N)));

% ---- 1) 3-D trajectory ---------------------------------------------------
fig(1) = figure('Name','3D trajectory','Color','w');
plot3(E, N, U, 'LineWidth', 1.8); hold on; grid on; axis equal;
scatter3(E(1),   N(1),   U(1),   80, 'g', 'filled');
scatter3(E(end), N(end), U(end), 80, 'r', 'filled');
patch([E(:);NaN], [N(:);NaN], [U(:);NaN], [tg(:);NaN], ...      % time-coloured
      'EdgeColor','interp','FaceColor','none','LineWidth',2.5);
colormap(turbo_safe); cb = colorbar; cb.Label.String = 'time [s]';
xlabel('East [m]'); ylabel('North [m]'); zlabel('Up [m]');
title('Drone 3-D trajectory (start = green, end = red)');
legend({'path','start','end'}, 'Location','best'); view(35,25);

% ---- 2) ground track -----------------------------------------------------
fig(2) = figure('Name','Ground track','Color','w');
plot(E, N, 'LineWidth', 1.8); hold on; grid on; axis equal;
plot(E(1), N(1), 'go', 'MarkerFaceColor','g', 'MarkerSize',9);
plot(E(end), N(end), 'ro', 'MarkerFaceColor','r', 'MarkerSize',9);
xlabel('East [m]'); ylabel('North [m]'); title('Ground track (top view)');
legend({'path','start','end'}, 'Location','best');

% ---- 3) altitude & speed -------------------------------------------------
fig(3) = figure('Name','Altitude and speed','Color','w');
subplot(2,1,1); plot(tg, U, 'LineWidth', 1.5); grid on;
ylabel('Altitude above start [m]'); title('Altitude'); hold on;
if ~isempty(D.odometry)                              % EKF estimate (NED -> up)
    O = D.odometry; plot(tsec(O), -O.z, '--', 'LineWidth', 1.2);
    legend({'GPS','EKF odometry'}, 'Location','best');
end
subplot(2,1,2); plot(tg, spd, 'LineWidth', 1.5); grid on;
ylabel('Horizontal speed [m/s]'); xlabel('Time [s]'); title('Ground speed');
if ismember('vel_d', G.Properties.VariableNames) && any(~isnan(G.vel_d))
    hold on; plot(tg, -G.vel_d, 'LineWidth', 1.2);
    legend({'horizontal','climb rate'}, 'Location','best');
end

% ---- 4) attitude ---------------------------------------------------------
if ~isempty(D.attitude)
    A = D.attitude; ta = tsec(A);
    [roll, pitch, yaw] = quat2euler_deg(A.qw, A.qx, A.qy, A.qz);
    fig(4) = figure('Name','Attitude','Color','w');
    subplot(3,1,1); plot(ta, roll,  'LineWidth', 1.3); grid on; ylabel('Roll [deg]');
    title('Attitude (from quaternion, NED/FRD)');
    subplot(3,1,2); plot(ta, pitch, 'LineWidth', 1.3); grid on; ylabel('Pitch [deg]');
    subplot(3,1,3); plot(ta, yaw,   'LineWidth', 1.3); grid on; ylabel('Yaw [deg]');
    xlabel('Time [s]');

    fig(5) = figure('Name','Body rates','Color','w');
    plot(ta, rad2deg([A.rollspeed A.pitchspeed A.yawspeed]), 'LineWidth', 1.2);
    grid on; xlabel('Time [s]'); ylabel('Rate [deg/s]');
    legend({'p (roll)','q (pitch)','r (yaw)'}); title('Body angular rates');
end

% ---- 5) battery ----------------------------------------------------------
if ~isempty(D.battery)
    B = D.battery; tb = tsec(B);
    fig(6) = figure('Name','Battery','Color','w');
    subplot(2,1,1); plot(tb, B.voltage_v, 'LineWidth', 1.5); grid on;
    ylabel('Voltage [V]'); title('Battery');
    subplot(2,1,2); plot(tb, B.current_a, 'LineWidth', 1.5); grid on;
    ylabel('Current [A]'); xlabel('Time [s]');
end

% ---- 6) mode / arming timeline ------------------------------------------
if ~isempty(D.status)
    S = D.status; ts = tsec(S);
    fig(7) = figure('Name','Flight modes','Color','w');
    subplot(2,1,1); stairs(ts, S.arming_state, 'LineWidth', 1.5); grid on;
    ylabel('arming\_state'); title('Arming state (1 = disarmed, 2 = armed)');
    subplot(2,1,2); stairs(ts, S.nav_state, 'LineWidth', 1.5); grid on;
    ylabel('nav\_state'); xlabel('Time [s]');
    title('PX4 navigation state (3=Mission, 4=Loiter, 5=RTL, 17=Takeoff, 18=Land)');
end

if saveFigs && ~isempty(logDir)
    for k = find(isgraphics(fig))
        exportgraphics(fig(k), fullfile(logDir, ...
            [strrep(lower(get(fig(k),'Name')),' ','_') '.png']), 'Resolution', 150);
    end
    fprintf('Figures saved to %s\n', logDir);
end
end

% ========================= helpers =======================================
function D = loadLogs(logDir)
names = {'gps','attitude','odometry','battery','status'};
D = struct();
if isempty(logDir)
    warning('No log folder given - using synthetic demo data.');
    D = demoData(); return;
end
logDir = char(java.io.File(logDir).getCanonicalPath());   % expand ~ etc.
for i = 1:numel(names)
    p = fullfile(logDir, [names{i} '.csv']);
    if isfile(p)
        T = readtable(p);
        if height(T) < 2, T = []; end
        D.(names{i}) = T;
    else
        D.(names{i}) = [];
        warning('Missing %s', p);
    end
end
end

function [r,p,y] = quat2euler_deg(w,x,y_,z)
n = sqrt(w.^2+x.^2+y_.^2+z.^2); w=w./n; x=x./n; y_=y_./n; z=z./n;
r = atan2(2*(w.*x + y_.*z), 1 - 2*(x.^2 + y_.^2));
p = asin(max(-1, min(1, 2*(w.*y_ - z.*x))));
y = atan2(2*(w.*z + x.*y_), 1 - 2*(y_.^2 + z.^2));
r = rad2deg(r); p = rad2deg(p); y = rad2deg(unwrap(y));
end

function cmap = turbo_safe
try, cmap = turbo(256); catch, cmap = jet(256); end
end

function D = demoData()
% Synthetic mission: take-off 5 m, square leg, return, land (mimics drone.cpp).
fs = 20; t = (0:1/fs:60)';
n = numel(t); E = zeros(n,1); N = E; U = E;
for k = 1:n
    tk = t(k);
    if tk < 8,        U(k) = 5*tk/8;
    elseif tk < 18,   U(k) = 5; N(k) = 10*(tk-8)/10;
    elseif tk < 28,   U(k) = 5; N(k) = 10; E(k) = 10*(tk-18)/10;
    elseif tk < 40,   U(k) = 5; N(k) = 10*(1-(tk-28)/12); E(k) = 10*(1-(tk-28)/12);
    else,             U(k) = max(0, 5*(1-(tk-40)/12));
    end
end
lat0 = 47.397742; lon0 = 8.545594; alt0 = 488e3;
us = int64(t*1e6) + int64(1e9);
vn = gradient(N,1/fs); ve = gradient(E,1/fs); vd = -gradient(U,1/fs);
D.gps = table(t, us, int32((lat0+N/111320)*1e7), ...
    int32((lon0+E/(111320*cosd(lat0)))*1e7), int32(alt0+U*1e3), vn, ve, vd, ...
    hypot(vn,ve), 3*ones(n,1), 10*ones(n,1), 'VariableNames', ...
    {'t_wall','px4_timestamp_us','lat_e7','lon_e7','alt_mm','vel_n','vel_e','vel_d','vel_m_s','fix_type','sats'});
roll  = 0.08*(gradient(E,1/fs)); pitch = -0.08*(gradient(N,1/fs));
yaw = 0.05*sin(t/5);
q = [cos(yaw/2).*cos(pitch/2).*cos(roll/2)+sin(yaw/2).*sin(pitch/2).*sin(roll/2), ...
     cos(yaw/2).*cos(pitch/2).*sin(roll/2)-sin(yaw/2).*sin(pitch/2).*cos(roll/2), ...
     cos(yaw/2).*sin(pitch/2).*cos(roll/2)+sin(yaw/2).*cos(pitch/2).*sin(roll/2), ...
     sin(yaw/2).*cos(pitch/2).*cos(roll/2)-cos(yaw/2).*sin(pitch/2).*sin(roll/2)];
D.attitude = table(t, us, q(:,1), q(:,2), q(:,3), q(:,4), gradient(roll,1/fs), ...
    gradient(pitch,1/fs), gradient(yaw,1/fs), 'VariableNames', ...
    {'t_wall','px4_timestamp_us','qw','qx','qy','qz','rollspeed','pitchspeed','yawspeed'});
D.odometry = table(t, us, N, E, -U, vn, ve, vd, 'VariableNames', ...
    {'t_wall','px4_timestamp_us','x','y','z','vx','vy','vz'});
D.battery = table(t, us, 12.6-0.01*t, 8+2*(U>0), 1-t/600, 'VariableNames', ...
    {'t_wall','px4_timestamp_us','voltage_v','current_a','remaining'});
nav = 17*(t<8) + 3*(t>=8 & t<40) + 5*(t>=40 & t<52) + 18*(t>=52);
D.status = table(t, us, 2*ones(n,1), nav, 'VariableNames', ...
    {'t_wall','px4_timestamp_us','arming_state','nav_state'});
end
