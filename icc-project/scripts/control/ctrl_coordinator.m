function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR Incremental actuator allocation for AFS, ESC, ABS and CDC.
% brakeTorque is added to the scenario brake torque by run_icc_scenario.
% Therefore negative values are intentionally allowed as ABS release commands;
% the runner clips the final physical brake torque to [0, MAX_BRAKE_TRQ].

rw = max(VEH.rw,0.1);
frontShare = 0.60;
rearShare  = 0.40;

% 1) Longitudinal incremental force -> wheel torque.
Fx = lonCmd.Fx_total;
if Fx < 0
    % Additional braking
    Tmag = -Fx*rw;
    Tlon = [frontShare*Tmag/2;
            frontShare*Tmag/2;
            rearShare*Tmag/2;
            rearShare*Tmag/2];
else
    % ABS pressure release: negative incremental brake torque
    Tmag = Fx*rw;
    Tlon = -[frontShare*Tmag/2;
             frontShare*Tmag/2;
             rearShare*Tmag/2;
             rearShare*Tmag/2];
end

% 2) ESC yaw moment -> left/right differential brake torque.
% Braking force produces yaw moment Mz = -y*Fx.
Mz = latCmd.yawMoment;
ratioF = 0.65;
hf = max(VEH.track_f/2,0.1);
hr = max(VEH.track_r/2,0.1);

% Differential axle force required for the requested yaw moment.
dFxF = ratioF*Mz/(2*hf);
dFxR = (1-ratioF)*Mz/(2*hr);

% Positive Mz: increase right-side brake and/or release left-side brake.
Tesc = [-dFxF*rw;
         dFxF*rw;
        -dFxR*rw;
         dFxR*rw];

% Limit ESC differential authority while retaining negative ABS release.
TescLim = 0.45*LIM.MAX_BRAKE_TRQ;
Tesc = max(-TescLim, min(TescLim,Tesc));

Tcmd = Tlon + Tesc;
Tcmd = max(-LIM.MAX_BRAKE_TRQ, min(LIM.MAX_BRAKE_TRQ,Tcmd));

% 3) AFS and CDC
actuatorCmd.steerAngle = max(-LIM.MAX_STEER_ANGLE, ...
                         min(LIM.MAX_STEER_ANGLE,latCmd.steerAngle));
actuatorCmd.brakeTorque = Tcmd;
actuatorCmd.dampingCoeff = max(CTRL.VER.cMin, ...
                           min(CTRL.VER.cMax,verCmd(:)));
end
