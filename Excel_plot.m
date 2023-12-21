
%% Plot excel file from GoLang server


%% 

% Specify the Excel file name and sheet name
file = 'lab_test_1.csv';
% sheet = 'Sheet1'; % Sheet name. For .xlsv

% Read the data from Excel
% data = xlsread(file, sheet); % For .xlsv
data = readtable(file);

% Extract x, y, and theta values
[rows, collums] = size(data);

x_t = data(:, end-2); %  x is the third last value in first collum
y_t = data(:, end-1); %  y is the second last value in first collum
theta_t = data(:, end); %  theta is the last value in first collum

% For plotting, from table to double
x = table2array(x_t); 
y = table2array(y_t);
theta = table2array(theta_t); 


theta_radians = deg2rad(theta);

%% Plotting


% Create a gradient for the arrows
gradient = linspace(0, 1, numel(theta));    

scale_factor = 10;
% Plotting with a gradient of brightness in a single color
figure;

% Plot lines connecting x-y positions
plot(x, y, '-', 'Color', 'b', 'LineWidth', 1);
hold on;

% Plot markers on top of the lines
plot(x, y, 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');

for i = 1:numel(theta)
    % Use HSV color space with varying brightness
    color = hsv2rgb([.5, 1, gradient(i)]);
    
    % Scale the vector components (cos(theta), sin(theta))
    dx = scale_factor * cos(theta_radians(i));
    dy = scale_factor * sin(theta_radians(i));
    
    quiver(x(i), y(i), dx, dy, 'Color', color, 'LineWidth', 1.5);
    
    fprintf('\n Brightness : %f', gradient(i));
%     fprintf('\n i : %f', i);

    hold on;
end

hold off;
title('2D position with theta');



%%%%%%%%%%%%%%%%%%%%%%%%%

% % theta as arrow
% figure;
% % plot(x, y, 'LineWidth', 1);
% quiver(x,y,cos(theta_radians), sin(theta_radians), 'Color', 'g', 'LineStyle', '-', 'marker', 'o');
% title('Theta Angle');


% Save as PNG in current directory, check directory with cmd 'pwd'
saveas(gcf, 'firkant_test_1_opti.png'); 



