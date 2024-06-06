%% Plot real position v estimate

% To save or not to save as png
% true - file saves to specified directory
% false - file is not saved

saveFile = true;

%% Load data 
% different names 
fileExcel = 'file.csv';
dataFileOptitrack = 'file.mat';

dataExcel = readtable(fileExcel);
load(dataFileOptitrack);

% % shared file name
% baseFileName = 'file';
% 
% % Define file names
% fileExcel = [baseFileName '.csv'];
% dataFileOptitrack = [baseFileName '.mat'];


%% Extract file name to use in saving as PNG 
% file is named according to excel file name
[~, basename, ~] = fileparts(fileExcel);



%% Optitrack, Real position
% Remove .mat and {}
loadedVariables = who('-file', dataFileOptitrack);
loadedVariableName = loadedVariables{1}; 

% Rename to data
dataOptitrack = eval(loadedVariableName);

% Reverse axis as needed
X = squeeze(dataOptitrack.Trajectories.Labeled.Data(:, 1, :))';
Y = squeeze(dataOptitrack.Trajectories.Labeled.Data(:, 2, :))';

%% Rotation of OptiTrack data 
% Insert angle of rotation for the Real poistion to line up. 
% If unwanted set to 0.

theta_deg = 0; % Rotation angle in degrees

% Convert to radians
theta_rad = theta_deg * (pi / 180); 

% Rotation matrix
R = [cos(theta_rad), -sin(theta_rad); sin(theta_rad), cos(theta_rad)];

% Original coordinates as column vectors
original_coords = [X(:), Y(:)]';

% Apply the rotation
rotated_coords = R * original_coords;

% Extract the rotated X and Y coordinates
X_rotated = rotated_coords(1, :);
Y_rotated = rotated_coords(2, :);



%% GoLang Excel, Estimate position

x_t = dataExcel(:, end-2); %  x is the third last value in first collum
y_t = dataExcel(:, end-1); %  y is the second last value in first collum
theta_t = dataExcel(:, end); %  theta is the last value in first collum

% For plotting, from table to double
% Optitrack data is in mm while Golang is in cm hence the *10
x = table2array(x_t) * 10; 
y = table2array(y_t) * 10;
theta = table2array(theta_t); 


%% Plot real position v estimate

% Set square size
squareSize = 0;

% Plot real position
figure;
% plot(X(:), Y(:), 'b-', 'LineWidth', 1); % Blue for real pos (OptiTrack)
plot(X_rotated, Y_rotated, 'b-', 'LineWidth', 1); % rotated real pos
hold on;

% Plot estimate position
plot(x, y, 'r-', 'LineWidth', 1); % Red for estimated (odometry)

squareCorners = [
    0, 0;              
    squareSize, 0;
    squareSize, squareSize;
    0, squareSize;
];

% Add markers at corners of the square
scatter(squareCorners(:, 1), squareCorners(:, 2), 100, 'black', 'Marker', 'o', 'LineWidth', 1);
% green marker at (0, 0)
scatter(0, 0, 100, 'g', 'Marker', 'o', 'LineWidth', 2); 

% labels and title
xlabel('X Position');
ylabel('Y Position');
title('Real vs. Estimated Position, CW');


% legend
hLegend = legend('Real Position', 'Estimated Position');
set(hLegend, 'Location', 'none');
% Adjust legend position
set(hLegend, 'Position', [0.5, 0.5, 0.1, 0.1]);  % Center of the plot

grid on;

axis auto;
% axis ([-100 1100 -100 1100]);
% axis ([-50 550 -50 550]);


hold off;

%% Save as PNG

if saveFile
    % Save in specified directory
    folder = 'C:\path\plots';
    if ~exist(folder, 'dir')
        mkdir(folder);
    end
    saveas(gcf, [folder '\' basename '_real_vs_estimated_position.png']);
end

% Save as PNG in current directory, check directory with cmd 'pwd'
% saveas(gcf, 'firkant.png'); 





