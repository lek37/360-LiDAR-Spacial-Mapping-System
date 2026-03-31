clc;
clear;
close all;

port = "COM3";
baud = 115200;
s = serialport(port, baud);
configureTerminator(s, "LF");
flush(s);

figure;
hold on;
grid on;
axis equal;
xlim([0 7000]);
ylim([-5000 5000]);
zlim([-3000 3000]);
xlabel('X Depth');
ylabel('Y Width');
zlabel('Z Height');
title('2DX3 Project Hallway Scan - Mark Le - 400558145');
view(3);

pointsPerLayer = 32;

% Current layer data, clear after 1 full layer
currentTheta = []; 
currentX = [];
currentY = [];
currentZ = [];

% Previous layer data, store it to connect the layers together
prevLayerX = [];
prevLayerY = [];
prevLayerZ = [];

fprintf('Listening on %s...\n', port); 

while true
    line = strtrim(readline(s));

    % Remove brackets if present
    line = erase(line, "[");
    line = erase(line, "]");

    vals = str2double(split(line, ",")); % Convert to numeric values
    % I want to take all data regardless of its rangeStatus
    if numel(vals) ~= 6 || any(isnan(vals))
        continue; 
    end

    theta = vals(1);
    rangeStatus = vals(2);
    r = vals(3);
    depth = vals(4);

    % print message showing received values, for debugging purpose!
    fprintf('Received values - Theta: %.2f, Range Status: %d, R: %.2f, Depth: %.2f\n', theta, rangeStatus, r, depth);

    % Take all data regardless of its rangeStatus
    if r <= 0
        continue;
    end

    x = depth;
    y = r * cosd(theta);
    z = r * sind(theta);

    currentTheta(end+1) = theta;
    currentX(end+1) = x;
    currentY(end+1) = y;
    currentZ(end+1) = z;

    % Once one full layer is collected
    if numel(currentX) == pointsPerLayer
        % Sort layer by angle
        [currentTheta, order] = sort(currentTheta);
        currentX = currentX(order);
        currentY = currentY(order);
        currentZ = currentZ(order);

        % Draw closed polygon for this layer
        plot3([currentX currentX(1)], ...
              [currentY currentY(1)], ...
              [currentZ currentZ(1)], ...
              '-o', 'LineWidth', 1.2);

        % Connect to previous layer
        if ~isempty(prevLayerX)
            for j = 1:pointsPerLayer
                plot3([prevLayerX(j) currentX(j)], ...
                      [prevLayerY(j) currentY(j)], ...
                      [prevLayerZ(j) currentZ(j)], ...
                      '-', 'LineWidth', 0.8);
            end
        end

        drawnow;

        fprintf('Layer at depth %.2f completed.\n', currentX(1));

        % Save current as previous
        prevLayerX = currentX;
        prevLayerY = currentY;
        prevLayerZ = currentZ;

        % Clear current layer buffers
        currentTheta = [];
        currentX = [];
        currentY = [];
        currentZ = [];
    end
end

clear s;
fprintf('Serial port closed.\n');

