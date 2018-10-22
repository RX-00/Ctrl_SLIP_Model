% sim_PassiveSLIP attempt at a passive SLIP model by Roy X.
% Find (x, y) of the point mass!
% 
% q = [ x, x dot, y, y dot]

clear; close all; clc

% input struct for all the chosen variables and parameters for the physics
% equations
input.theta = 8 * pi / 16-.1;
assert(input.theta < pi, 'ERROR: Touchdown theta must not be greater than pi')
input.d0 = .9; % Changed dDef to d0 since it's just better notation
input.k = 4500;
input.m = 20;
input.g = 9.81;
input.d_fwrd_vel = 0.5;

% Starting conditions of the state vector x, fwrd vel, y, upwrd vel,
% foot position upon touchdown, and what phase you're in (0 for flight, 1
% for stance)
q0 = [0; .5; 1.2; 0; 0; 0];

%-------------------------------------------------------------------- 
% TODO: Step through all the dynamics & events to make sure x touch-
% down isn't conserved, or something other weird thing is going on
%--------------------------------------------------------------------

refine = 4;

flightEvent = @(t, q) flightToStance(t, q, input);
stanceEvent = @(t, q) stanceToFlight(t, q, input);

% due to the plot here ode plots everytime by itself, helps see what's
% going on inbetween
optionsFlight = odeset('Events', flightEvent, 'OutputFcn', @odeplot, 'OutputSel', 1, ...
    'Refine', refine);
optionsStance = odeset('Events', stanceEvent, 'OutputFcn', @odeplot, 'OutputSel', 1, ...
    'Refine', refine);

% time stuff
tspan = [0 5];
tStep = 0.009;
tstart = tspan(1);
tend = tspan(end);
twhile = tstart; % global solution time

tout = []; % gather up time
qout = []; % gather up state vectors
KEout = []; % gather up kinetic energy
PEout = []; % gather up potential energy

teout = []; % time when events occur
qeout = []; % state when events occur
ieout = []; % phase which event trigger the switch

% Flight function
flightDyn = @(t, q) SLIP_Flight(t, q, input);

% Stance function
stanceDyn = @(t, q) SLIP_Stance(t, q, input);

% Old tests of stance and flight isolated
%[t, q] = ode45(stanceDyn, tspan, q0);
%[t, q] = ode45(flightDyn, tspan, q0);

bounce_num = 0;

while isempty(tout) || tout(end) < tend - tStep
    if q0(6) == 0 % The model is in flight phase until transition to stance phase is triggered       

        optionsFlight = odeset('Events', flightEvent, 'OutputFcn', @odeplot, 'OutputSel', 1, ...
    'Refine', refine);
        [t, q, te, qe, ie] = ode45(flightDyn, [tstart(end):tStep:tend], q0, optionsFlight);
        
        tstart = t;
        % forward foot placement
        q(end, 5) = q(end,1) - input.d0 * cos(pi - input.theta); % based on chosen theta
        q(end, 6) = 1;
        q0 = q(end,:);
        bounce_num = bounce_num + 1; % you can't do ++ in Matlab??!! smh

        
        % Accumulate output
        nt = length(t);
        tout = [tout; t(2:nt)];
        qout = [qout; q(2:nt,:)];
        teout = [teout; te];
        qeout = [qeout; te];
        ieout = [ieout; te];
        
        % Check if everything is alright, i.e. not y < 0
        if q(end, 3) <= 0
            % Terminate the program for the SLIP model has fallen
            fprintf('SLIP Model has fallen (y < 0) at t = %f \n', tout(end))
            break;
        end
        
    else % The model is in stance phase until transition to flight phase is triggered
        
        optionsStance = odeset('Events', stanceEvent, 'OutputFcn', @odeplot, 'OutputSel', 1, ...
    'Refine', refine);
        [t, q, te, qe, ie] = ode45(stanceDyn, [tstart(end):tStep:tend], q0, optionsStance);

        tstart = t;
        q(end, 6) = 0;
        q0 = q(end,:);    
            
        % RAIBERT P CONTROLLER
        [xf, theta] = raibertPController(q, input, t);
        input.theta = theta;

        
        % Accumulate output
        nt = length(t);
        tout = [tout; t(2:nt)];
        qout = [qout; q(2:nt,:)];
        teout = [teout; te];
        qeout = [qeout; te];
        ieout = [ieout; te];
        
        % Check if everything is alright, i.e. not y < 0
        if q(end, 3) <= 0
            % Terminate the program for the SLIP model has fallen
            fprintf('SLIP Model has fallen (y < 0) at t = %f \n', tout(end))
            break;
        end
    end
    
end



plot(qout(:,1), qout(:,3));
fprintf('Bounced %d times \n', bounce_num)

xlabel('distance');
ylabel('height');
title('SLIP Model COM Trajectory');
hold off


% Calculate Energy in the system
for i = 1:(length(tout)) % the last value is when the SLIP has fallen
    if i == 141
        1+1;
    end
    if(qout(i, 6) == 1) % energy in stance
        d = sqrt((qout(i, 1) - qout(i, 5)).^2 + (qout(i, 3).^2));
        KEout = [KEout; 1 / 2 * input.m * ((qout(i, 2)).^2 + (qout(i, 4)).^2)];
        PEout = [PEout; input.m * input.g * qout(i, 3) + 1/2 * input.k * (input.d0 - d).^2];
    else % energy in flight
        KEout = [KEout; 1 / 2 * input.m * ((qout(i, 2)).^2 + (qout(i, 4)).^2)];
        PEout = [PEout; input.m * input.g * qout(i, 3)];
    end
end


% PLOT ALL OF YOUR DATA
figure(50);
plot(tout, KEout);
hold on;
plot(tout, PEout);
hold on;
plot(tout, PEout + KEout);
for i = 1:(length(teout))
    line([teout(i) teout(i)], [0 300]);
end
legend('Kinetic','Potential','Total','trigger event time');
hold on;

animate_SLIP(qout, input, tout);