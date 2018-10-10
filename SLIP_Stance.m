function dy = SLIP_Stance(t, q, s)
% SLIP_Stance attempt at a passive SLIP model by Roy X.
% Find (x, y) of the point mass!
% This is the stance phase
% s stands for the struct of all the parameters
%
% q = [ x, x dot, y, y dot, ft pos, phase]
% NOTE: changed the y state vector name to q just for clarity
% Thus, in order to define x and y you must do
% x = y(:,1), y = y(:,3)
    
    x = q(1);
    y = q(3);
    
    xtd = q(5);
    ytd = 0;
  
    % Functions that describe the motion of the SLIP Model's COM
    d = sqrt((x - xtd)^2 + (y)^2)
    assert(d ~= 0, 'COM distance from zero must not be zero')
    sinT = y / d;
    x0 = (s.d0 * cos(pi - s.theta));
    cosT =  (x - xtd)/ d;
    Fs = s.k * (s.d0 - d);
    Fy = Fs * sinT;
    Fx = Fs * cosT;
    Fyt = Fy - s.m * s.g;
        
    dy(1, 1) = q(2); % x dot
    dy(2, 1) = (Fx / s.m); % x double dot
    dy(3, 1) = q(4); % y dot
    dy(4, 1) = (Fyt / s.m); % y double dot
    dy(5, 1) = 0; % foot position upon touchdown
    dy(6, 1) = 0; % what phase you're in, but you don't want to set the value here or else it will be part of the integration
end