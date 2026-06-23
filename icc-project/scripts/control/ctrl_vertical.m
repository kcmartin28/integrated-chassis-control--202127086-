function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL Smoothed on-off skyhook CDC controller.

% For models without vertical states, return a safe passive setting.
required = {'zs_dot','zu_dot'};
if ~all(isfield(suspState,required))
    dampingCmd = 1500*ones(4,1);
    return;
end

zsDot = suspState.zs_dot(:);
zuDot = suspState.zu_dot(:);
relVel = zsDot - zuDot;

% On-off skyhook rule with a continuous transition near zero.
switchSignal = zsDot .* relVel;
epsSwitch = 2.0e-4;
blend = 0.5*(1 + tanh(switchSignal/epsSwitch));

cTarget = CTRL.VER.cMin + ...
          blend.*(CTRL.VER.cMax-CTRL.VER.cMin);

% Mild groundhook contribution for wheel-hop suppression.
wheelActivity = abs(zuDot) ./ (abs(zuDot)+0.08);
cTarget = (1-0.25*wheelActivity).*cTarget + ...
          (0.25*wheelActivity).*CTRL.VER.skyGain;

cTarget = max(CTRL.VER.cMin, min(CTRL.VER.cMax, cTarget));

% First-order command smoothing
if ~isfield(ctrlState,'cPrev') || numel(ctrlState.cPrev) ~= 4
    ctrlState.cPrev = 1500*ones(4,1);
end
tauC = 0.02;
alpha = dt/(tauC+dt);
dampingCmd = ctrlState.cPrev + alpha*(cTarget-ctrlState.cPrev);
dampingCmd = max(CTRL.VER.cMin, min(CTRL.VER.cMax, dampingCmd));
ctrlState.cPrev = dampingCmd;
end
