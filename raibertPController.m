function [xf, theta] = raibertPController(q, s, t)
%RAIBERTCONTROLLER Simple Raibert proportional controller for SLIP monoped
%   This controller is called at lift off (entering flight phase)
%   in order to adjust the leg touchdown angle to maintain
%   the desired forward velocity

    % This is the feedback gain for ft displacement from neutral point
    % measured in m/(m/s), which is just s
    k = 0.05;
    xf = 0;
    theta = 0;
    
    if q(end, 6) == 0 % If the model is in flight phase

        %-------------------------------------------------------------------- 
        % TODO: How would I find the p0 of the proportional controller,
        % i.e. the controller output w/ no error
        %--------------------------------------------------------------------

        
        xf = 0.5 * q(end, 2) * 0.225 + k * (q(end, 2) - s.d_fwrd_vel);
        
        thetaLegBody = asin(xf / s.d0);
        
        theta = thetaLegBody + pi / 2;
        
    end
    
    % If the model is in stance, the controller does not need to do
    % anything (for this SLIP model at least)    
end