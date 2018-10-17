function [valFs, isterminal, direction] = stanceToFlight(t, q, s)
% STANCETOFLIGHT Detect when stance changes to flight
%   If the spring force is zero then you know you have left the ground,
%   thus transitioning into the flight phase
    d = sqrt((q(1) - q(5))^2 + (q(3))^2);
    valFs = [s.k * (s.d0 - d), q(3)]; % if the spring force is 0 then you've left the ground
    isterminal = [1, 1]; % stops the integration
    
    direction = [-1, ];
end