function [split] = splitDirectionFun(positionData, plotResults)
%splitDirectionFun Takes in position data from a 3D trajectory and fits a
%line to the trajectory data by computing the
%principal component of movement direction, and returns the estimate of the timestamp when the
%direction changes

% Smoothing the pose data (using a moving average)
windowSize = 3000; % Adjust based on your data
trajectory = movmean(positionData, windowSize);

% Compute the mean of the points
meanPoint = mean(trajectory, 1);

% Center the data by subtracting the mean
centeredData = trajectory - meanPoint;

% Perform PCA using Singular Value Decomposition (SVD)
[U, S, V] = svd(centeredData, 'econ');

% The first principal component (direction of the line)
principalDirection = V(:, 1);

% Project the data onto the principal component
projections = centeredData * principalDirection;

% Analyze the progression of projections to identify direction changes
directionChange = diff(projections); % Positive: forward, Negative: backward
threshold = 1e-7; % Threshold to ignore very small changes
directionChangeSmoothed = movmean(directionChange, 2500);
forwardSegments = find(directionChange > threshold);
backwardSegments = find(directionChange < -threshold);

split = backwardSegments(1);
if backwardSegments(1) < forwardSegments(1)
    error('ValueCheck:NegativeInput', 'forwardSegment not parsed properly (backwards starts first)');

end

if plotResults
    % Visualize the trajectory with forward and backward segments
    figure;
    subplot(2,1,1);
    plot3(trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), 'k-', 'DisplayName', 'Original Trajectory');
    hold on;
    
    % Highlight forward and backward segments
    scatter3(trajectory(forwardSegments, 1), trajectory(forwardSegments, 2), trajectory(forwardSegments, 3), 1, 'g','filled' , 'DisplayName', 'Forward');
    scatter3(trajectory(backwardSegments, 1), trajectory(backwardSegments, 2), trajectory(backwardSegments, 3),1, 'r', 'filled', 'DisplayName', 'Backward');
    
    % % % Add the linear approximation for reference
    % t = linspace(-1, 1, 100)'; % Parameter for the line
    % linePoints = meanPoint + t * principalDirection';
    % plot3(linePoints(:, 1), linePoints(:, 2), linePoints(:, 3), 'b-', 'LineWidth', 2, 'DisplayName', 'Linear Approximation');
    
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Filtered Trajectory with Forward and Backward Segments');
    legend('show');
    grid on;
    axis equal
    
    subplot(2,1,2)
    % Highlight forward and backward segments
    plot3(positionData(:, 1), positionData(:, 2), positionData(:, 3), 'k-', 'DisplayName', 'Original Trajectory');
    hold on;
    scatter3(positionData(forwardSegments, 1), positionData(forwardSegments, 2), positionData(forwardSegments, 3), 1, 'g','filled' , 'DisplayName', 'Forward');
    scatter3(positionData(backwardSegments, 1), positionData(backwardSegments, 2), positionData(backwardSegments, 3),1, 'r', 'filled', 'DisplayName', 'Backward');
    title('Raw Trajectory with Forward and Backward Segments');
    legend('show');
    grid on
    axis equal
end



end