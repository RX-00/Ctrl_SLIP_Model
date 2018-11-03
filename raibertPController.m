function [xf, theta] = raibertPController(q, s, t)
%RAIBERTCONTROLLER Simple Raibert proportional controller for SLIP monoped
%   This controller is called at lift off (entering flight phase)
%   in order to adjust the leg touchdown angle to maintain
%   the desired forward velocity

    % This is the feedback gain for ft displacement from neutral point
    % measured in m/(m/s), which is just s
    k = -0.0595; %got to around the fraction -43/726 and then called it quits
    
    % NOTE: for tuning the gain there's not a general "rule" for
    % approaching a number, try random numbers and then tune from the best
    % there
    k = -0.2;
    xf = 0;
    theta = 0;
    
    if q(end, 6) == 0 % If the model is in flight phase
        
        %theta = s.theta + k * (q(end, 2) - s.d_fwrd_vel);
        theta = pi / 2 + k * (q(end, 2) - s.d_fwrd_vel);
        
        xf = cos(theta) / s.d0;
        
    end
    
    % If the model is in stance, the controller does not need to do
    % anything (for this SLIP model at least)    
end