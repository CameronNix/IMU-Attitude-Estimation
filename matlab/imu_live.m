clear all
dt = 0.01;
q = 0.01;
s = serialport("COM3", 115200);
configureTerminator(s, "LF");
flush(s);

% --- Gyro bias calibration ---
gyroBiasX = 0; gyroBiasY = 0; gyroBiasZ = 0;
for i = 1:500
    line = readline(s);
    values = str2double(split(line, ","));
    gyroBiasX = gyroBiasX + values(5);
    gyroBiasY = gyroBiasY + values(6);
    gyroBiasZ = gyroBiasZ + values(7);
end
gyroBiasX = gyroBiasX / 500;
gyroBiasY = gyroBiasY / 500;
gyroBiasZ = gyroBiasZ / 500;

% --- Calibration: Pitch ---
calibData = zeros(500,1);
for i = 1:500
    line = readline(s);
    values = str2double(split(line, ","));
    aX = values(2); aZ = values(4);
    calibData(i) = atan2d(aX, aZ);
end
Rpitch = var(calibData);

% --- Calibration: Roll ---
calibData = zeros(500,1);
for i = 1:500
    line = readline(s);
    values = str2double(split(line, ","));
    aY = values(3); aZ = values(4);
    calibData(i) = atan2d(aY, aZ);
end
Rroll = var(calibData);

% --- Calibration: Yaw ---
calibData = zeros(500,1);
for i = 1:500
    line = readline(s);
    values = str2double(split(line, ","));
    mY = values(9); mX = values(8);
    calibData(i) = atan2d(mY, mX);
end
Ryaw = var(calibData);

% --- Initialize filter state ---
line = readline(s);
values = str2double(split(line, ","));
aX = values(2); aY = values(3); aZ = values(4);
mX = values(8); mY = values(9);

xPitch = atan2d(aX, aZ);
pPitch = 1;
xRoll = atan2d(aY, aZ);
pRoll = 1;
xYaw = atan2d(mY, mX);
pYaw = 1;

% --- Set up live plots: 3 stacked subplots ---
bufferSize = 500;
plotTime = nan(1, bufferSize);
plotPitch = nan(1, bufferSize);
plotAccelAnglePitch = nan(1, bufferSize);
plotRoll = nan(1, bufferSize);
plotAccelAngleRoll = nan(1, bufferSize);
plotYaw = nan(1, bufferSize);
plotMagAngleYaw = nan(1, bufferSize);
idx = 1;

figure('Position', [100 100 700 800])

subplot(3,1,1)
hP1 = plot(NaN, NaN, 'r', 'LineWidth', 1.5);
hold on
hP2 = plot(NaN, NaN, 'b');
hold off
ylabel('Pitch (deg)')
legend('Filtered', 'Raw')
ylim([-180 180])
title('Pitch')

subplot(3,1,2)
hR1 = plot(NaN, NaN, 'r', 'LineWidth', 1.5);
hold on
hR2 = plot(NaN, NaN, 'b');
hold off
ylabel('Roll (deg)')
ylim([-180 180])
title('Roll')

subplot(3,1,3)
hY1 = plot(NaN, NaN, 'r', 'LineWidth', 1.5);
hold on
hY2 = plot(NaN, NaN, 'b');
hold off
ylabel('Yaw (deg)')
xlabel('Time (s)')
ylim([-180 180])
title('Yaw')

% --- Main live loop ---
while true
    line = readline(s);
    values = str2double(split(line, ","));
    t_ms = values(1);
    aX = values(2); aY = values(3); aZ = values(4);
    gX = values(5) - gyroBiasX;
    gY = values(6) - gyroBiasY;
    gZ = values(7) - gyroBiasZ;
    mX = values(8); mY = values(9); mZ = values(10);

    AccelAnglePitch = atan2d(aX, aZ);
    AccelAngleRoll = atan2d(aY, aZ);
    MagAngleYaw = atan2d(mY, mX);

    x_pred = xPitch + gX * dt;
    p_pred = pPitch + q;
    Kpitch = p_pred / (p_pred + Rpitch);
    xPitch = x_pred + Kpitch * (AccelAnglePitch - x_pred);
    pPitch = (1 - Kpitch) * p_pred;

    x_pred = xRoll + gY * dt;
    p_pred = pRoll + q;
    Kroll = p_pred / (p_pred + Rroll);
    xRoll = x_pred + Kroll * (AccelAngleRoll - x_pred);
    pRoll = (1 - Kroll) * p_pred;

    x_pred = xYaw + gZ * dt;
    p_pred = pYaw + q;
    Kyaw = p_pred / (p_pred + Ryaw);
    xYaw = x_pred + Kyaw * (MagAngleYaw - x_pred);
    pYaw = (1 - Kyaw) * p_pred;

    xPitch = mod(xPitch + 180, 360) - 180;
    xRoll  = mod(xRoll + 180, 360) - 180;
    xYaw   = mod(xYaw + 180, 360) - 180;

    plotTime(idx) = t_ms / 1000;
    plotPitch(idx) = xPitch;
    plotAccelAnglePitch(idx) = AccelAnglePitch;
    plotRoll(idx) = xRoll;
    plotAccelAngleRoll(idx) = AccelAngleRoll;
    plotYaw(idx) = xYaw;
    plotMagAngleYaw(idx) = MagAngleYaw;

    nextIdx = mod(idx, bufferSize) + 1;
    plotTime(nextIdx) = NaN;
    plotPitch(nextIdx) = NaN;
    plotAccelAnglePitch(nextIdx) = NaN;
    plotRoll(nextIdx) = NaN;
    plotAccelAngleRoll(nextIdx) = NaN;
    plotYaw(nextIdx) = NaN;
    plotMagAngleYaw(nextIdx) = NaN;
    idx = nextIdx;

    set(hP1, 'XData', plotTime, 'YData', plotPitch)
    set(hP2, 'XData', plotTime, 'YData', plotAccelAnglePitch)
    set(hR1, 'XData', plotTime, 'YData', plotRoll)
    set(hR2, 'XData', plotTime, 'YData', plotAccelAngleRoll)
    set(hY1, 'XData', plotTime, 'YData', plotYaw)
    set(hY2, 'XData', plotTime, 'YData', plotMagAngleYaw)

    drawnow limitrate
end
