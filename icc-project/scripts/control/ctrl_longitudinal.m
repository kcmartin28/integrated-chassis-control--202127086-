function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL Incremental speed/ABS controller.
%   Negative Fx_total requests additional braking.
%   Positive Fx_total is used as an incremental brake-release request because
%   run_icc_scenario adds controller brake torque to the scenario brake torque.

if ~isfield(ctrlState,'intError'); ctrlState.intError = 0; end
if ~isfield(ctrlState,'prevForce'); ctrlState.prevForce = 0; end
if ~isfield(ctrlState,'wheelSlip'); ctrlState.wheelSlip = zeros(4,1); end

eV = vxRef - vx;
mEst = 1500;  % repository nominal vehicle mass [kg]

% PI speed loop. In benchmark braking scenarios the reference stays at vx0,
% so this mainly prevents unnecessary additional braking.
intCandidate = ctrlState.intError + eV*dt;
intCandidate = max(-CTRL.LON.intMax, min(CTRL.LON.intMax, intCandidate));

FxPI = CTRL.LON.Kp*mEst*eV + CTRL.LON.Ki*mEst*intCandidate;

% ABS release based on previous-step wheel slips supplied by the runner.
kappa = abs(ctrlState.wheelSlip(:));
kTarget = 0.10;
kHigh   = 0.12;
kPeak   = max(kappa);

releaseForce = 0;
if ax < -0.2 && kPeak > kHigh
    % Proportional pressure-release request.
    releaseForce = 2.5e4 * (kPeak - kTarget);
    releaseForce = min(releaseForce, 1.2*mEst*9.81);
end

% Do not add braking merely because speed fell below the fixed initial target.
% Positive output is interpreted by the coordinator as brake release.
if releaseForce > 0
    FxDesired = releaseForce;
else
    FxDesired = min(FxPI, 0);
end

% Jerk limiter on incremental longitudinal force
dFmax = mEst * LIM.MAX_JERK * dt;
FxCmd = max(ctrlState.prevForce-dFmax, ...
        min(ctrlState.prevForce+dFmax, FxDesired));

% Conditional anti-windup
if abs(FxDesired-FxCmd) < 1e-9
    ctrlState.intError = intCandidate;
end

forceCmd.Fx_total = FxCmd;
forceCmd.brakeRatio = min(max(kPeak/kHigh,0),1);
ctrlState.prevForce = FxCmd;
end
