function [] = animate_SLIP(q, s, t)
%ANIMATE_SLIP animate the SLIP model
%   Take in the state vector and forces to model the SLIP model

    figure(100); % TODO: Figure out this whole animation process thing
    cla
    xlim([-3.5, 3.5]);
    ylim([-0.3, 2]);
    grid on;
    axis equal;
    title('SLIP Animation');
    xlabel('distance');
    ylabel('height');
    
    % used to calculate the leg length
    x = q(1, 1);
    y = q(1, 3);
    xtd = q(1, 5);
    ytd = 0;
    d = sqrt((x - xtd)^2 + (y)^2);
    
    % Visual patches
    ground_patch = patch([-100, -100, 100, 100], [0, -10, -10, 0], [0.5, 0.5, 0.5]);
    
    body_patch = patch(q(1, 1) + 0.1 * sin(0: 0.1: 2 * pi), q(1, 3) + 0.1 * cos(0: 0.1: 2 * pi), [70, 216, 226]./255);
    
    startTheta = s.theta - pi / 2;
    leg_patch = patch(q(1, 1) + [0.01,0.01,-0.01,-0.01] * cos(startTheta) + s.d0 * [0,1,1,0] * sin(startTheta),...);
         q(1, 3) + [0.01,0.01,-0.01,-0.01] * sin(startTheta) + s.d0 * [0,-1,-1,0] * cos(startTheta), 'k');

    drawnow;
    
    % Loop through the data and update the graphics
    for i = 1:(length(t) - 1) % minus one in order to not animate the last part of the data that changes from stance (fallen) to flight (even though irl that is impossible)
        
        body_patch.Vertices = [q(i, 1) + 0.1 * sin(0: 0.1: 2 * pi); q(i, 3) + 0.1 * cos(0: 0.1: 2 * pi)]';
        
        % used to calculate the leg length
        x = q(i, 1);
        y = q(i, 3);
        xtd = q(i, 5);
        ytd = 0;
        d0 = s.d0;
        d = sqrt((x - xtd)^2 + (y)^2);
         
        
        % NOTE: This algorithm was originally for pitch angle of the leg
        % from the body and so in order to use it with your touchdown angle
        % (right side angle of leg touching ground) you need to add
        % pi / 2
        % NOTE: This line uses the pitch angle from if the leg was straight
        % up and down to where the leg actually is
        %leg_patch.Vertices = [q(i, 1) + [0.01,0.01,-0.01,-0.01] * cos(inputTheta) + d * [0,1,1,0] * sin(inputTheta);...);
                       %q(i, 3) + [0.01,0.01,-0.01,-0.01] * sin(inputTheta) + d * [0,-1,-1,0] * cos(inputTheta)]';
        
        if(q(i, 6) == 1)
            inputTheta = asin((xtd - x) / d);
            leg_patch.Vertices = [q(i, 1) + [0.01,0.01,-0.01,-0.01] * cos(inputTheta) + d * [0,1,1,0] * sin(inputTheta);...);
                       q(i, 3) + [0.01,0.01,-0.01,-0.01] * sin(inputTheta) + d * [0,-1,-1,0] * cos(inputTheta)]';
        else % If it is in flight lift leg up and be d0
            inputTheta = s.theta - (pi / 2);
            if(q(i, 2) < 0)
                inputTheta = -inputTheta;
            end            
            leg_patch.Vertices = [q(i, 1) + [0.01,0.01,-0.01,-0.01] * cos(inputTheta) + d0 * [0,1,1,0] * sin(inputTheta);...);
                       q(i, 3) + [0.01,0.01,-0.01,-0.01] * sin(inputTheta) + d0 * [0,-1,-1,0] * cos(inputTheta)]';
        end                   
                   
  
        ylim([-2, 2]);
        
        % Increment the screen by 0.5 m increments
        %xlim([-1.5, 1.5] + round(q(i, 1) * 2) / 2);
        
        drawnow;
        %pause(0.1);
    end
end

