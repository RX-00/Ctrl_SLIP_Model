function [xf, theta] = raibertPController(q, s, t)
%RAIBERTCONTROLLER Simple Raibert proportional controller for SLIP monoped
%   This controller is called at lift off (entering flight phase)
%   in order to adjust the leg touchdown angle to maintain
%   the desired forward velocity

    % This is the feedback gain for ft displacement from neutral point
    % measured in m/(m/s), which is just s
    k = -.12531;
    xf = 0;
    theta = 0;
    
    if q(end, 6) == 0 % If the model is in flight phase
        
        theta = s.theta + k * (q(end, 2) - s.d_fwrd_vel)
        
        xf = cos(theta) / s.d0;
        
    end
    
    % If the model is in stance, the controller does not need to do
    % anything (for this SLIP model at least)    
end