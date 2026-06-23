function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL Gain-scheduled AFS + ESC controller.
%   AFS: filtered PID yaw-rate tracking with conditional integration.
%   ESC: sideslip/yaw-error feedback producing a corrective yaw moment.

% State initialization
if ~isfield(ctrlState,'intError');      ctrlState.intError = 0; end
if ~isfield(ctrlState,'prevError');     ctrlState.prevError = 0; end
if ~isfield(ctrlState,'dErrorFilt');    ctrlState.dErrorFilt = 0; end

% Yaw-rate tracking error
e = yawRateRef - yawRate;

% Speed scheduling: reduce steering aggressiveness at high speed
vSched = min(max(vx / 20.0, 0.35), 1.50);
kp = CTRL.LAT.Kp / vSched;
ki = CTRL.LAT.Ki / vSched;
kd = CTRL.LAT.Kd / vSched;

% Filtered derivative to suppress numerical noise
tauD = 0.03;
dRaw = (e - ctrlState.prevError) / max(dt,1e-6);
alphaD = dt / (tauD + dt);
ctrlState.dErrorFilt = ctrlState.dErrorFilt + ...
    alphaD * (dRaw - ctrlState.dErrorFilt);

% Candidate integral
intCandidate = ctrlState.intError + e*dt;
intCandidate = max(-CTRL.LAT.intMax, min(CTRL.LAT.intMax, intCandidate));

% Unsaturated AFS command
uUnsat = kp*e + ki*intCandidate + kd*ctrlState.dErrorFilt;

% Limit the auxiliary steering to a conservative fraction of total authority
afsLimit = min(0.30*LIM.MAX_STEER_ANGLE, deg2rad(5.0));
uSat = max(-afsLimit, min(afsLimit, uUnsat));

% Conditional-integration anti-windup
satHigh = (uUnsat >  afsLimit) && (e > 0);
satLow  = (uUnsat < -afsLimit) && (e < 0);
if ~(satHigh || satLow)
    ctrlState.intError = intCandidate;
end

deltaAdd.steerAngle = uSat;

% ESC: activate before the absolute safety limit.
betaTh = min(deg2rad(3.0), 0.40*LIM.MAX_SLIP_ANGLE);
betaExcess = max(abs(slipAngle) - betaTh, 0);

% Speed-dependent corrective authority
fV = min(max(vx/15.0, 0.0), 2.0);

% Sideslip term dominates; yaw-error term adds damping.
Kbeta = 4.0e4;          % [Nm/rad]
Kr    = 4.0e3;          % [Nms/rad]
MzBeta = -Kbeta * sign(slipAngle) * betaExcess * fV;

% Yaw-rate ESC를 sideslip 크기에 따라 부드럽게 활성화
betaOn   = deg2rad(1.5);  % 이 값 이하에서는 ESC yaw feedback 비활성화
betaFull = deg2rad(3.0);  % 이 값 이상에서는 완전히 활성화

escBlend = (abs(slipAngle) - betaOn) / ...
           max(betaFull - betaOn, 1e-6);

% escBlend를 0~1 범위로 제한
escBlend = max(0, min(1, escBlend));

% 부드럽게 증가하는 yaw-rate feedback moment
MzYaw = escBlend * Kr * e * min(fV, 1.5);

% Suppress tiny brake interventions near straight driving
if abs(slipAngle) < deg2rad(0.5) && abs(e) < deg2rad(0.8)
    MzCmd = 0;
else
    MzCmd = MzBeta + MzYaw;
end

MzLimit = 4500;         % conservative brake-generated yaw moment limit
deltaAdd.yawMoment = max(-MzLimit, min(MzLimit, MzCmd));

ctrlState.prevError = e;
end
