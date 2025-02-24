%% DWA with Global Re-planning, Global Mode Timeout, and Moving Obstacles
% This script simulates a robot that uses DWA for local planning. If the robot
% becomes stuck, a global planner (A* on the grid) is invoked to compute an alternate
% path. Dynamic obstacles (moving obstacles) are added to the environment.
% The combined grid (static maze + moving obstacles) is used for planning.

clear; clc; close all;

%% Environment Setup - Static Maze
gridSize = [20,20];      % Grid dimensions (rows, columns)
grid_static = zeros(gridSize);  % 0 = free cell, 1 = obstacle

% Outer walls
grid_static(1,:) = 1;        % Top wall
grid_static(20,:) = 1;       % Bottom wall
grid_static(:,1) = 1;        % Left wall
grid_static(:,20) = 1;       % Right wall

% Internal walls to form a maze:
% 1. Vertical wall at column 4 from rows 2 to 10 with a gap from rows 5 to 7
grid_static(2:10,4) = 1;
grid_static(5:7,4) = 0;  % Larger gap

% 2. Horizontal wall at row 4 from columns 4 to 12 with a gap from columns 7 to 9
grid_static(4,4:12) = 1;
grid_static(4,7:9) = 0;  % Larger gap

% 3. Vertical wall at column 12 from rows 4 to 15 with a gap from rows 9 to 11
grid_static(4:15,12) = 1;
grid_static(9:11,12) = 0;  % Larger gap

% 4. Horizontal wall at row 15 from columns 4 to 12 with a gap from columns 6 to 8
grid_static(15,4:12) = 1;
grid_static(15,6:8) = 0;  % Larger gap

% 5. Additional vertical wall at column 8 from rows 8 to 15 with a gap from rows 11 to 13
grid_static(8:15,8) = 1;
grid_static(11:13,8) = 0;

%% Define Dynamic (Moving) Obstacles
% Each moving obstacle has a continuous position [x, y] and velocity [vx, vy].
% Their positions will be updated every iteration.
movingObstacles(1).pos = [10, 5];    % Starting position (x,y)
movingObstacles(1).vel = [0.1, 0.05];  % Moves rightward and upward
movingObstacles(2).pos = [15, 10];     
movingObstacles(2).vel = [-0.1, 0.05]; % Moves leftward and upward

%% Robot and Simulation Parameters
start = [2, 2, 0];      % Starting state [x, y, theta]
main_goal  = [18, 18];  % Main goal position [x, y]

% DWA and robot parameters
params.max_v = 1.0;              % Maximum linear velocity [m/s]
params.max_omega = 1.0;          % Maximum angular velocity [rad/s]
params.dt = 0.1;                 % Time step [s]
params.predict_time = 3.0;       % Prediction horizon [s]
params.velocity_resolution = 0.1;  
params.omega_resolution = 0.1;     
params.robot_radius = 0.5;       

% LIDAR parameters
lidar.angles = -pi/3 : pi/36 : pi/3;  
lidar.max_range = 5;                  

%% Global Planning Variables
global_mode = false;    % Flag for using the global planner
global_path = [];       % List of waypoints (grid cells) from global planner
subgoal_index = 1;      % Index of the current subgoal along the global path
subgoal_threshold = 0.7;  % Distance threshold (in grid units) for subgoal attainment

% Global mode iteration counter for timeout
global_mode_iter_count = 0;
global_mode_timeout = 20;  % After 10 iterations in global mode, revert to local planning

%% Simulation Loop Variables
state = start;             
trajectory = state(1:2);   
max_iterations = 1000;

% Variables to detect stagnation (lack of progress)
stuck_counter = 0;
last_distance = norm(state(1:2) - main_goal);
progress_threshold = 0.05;  
stuck_limit = 15;           

figure;
for iter = 1:max_iterations
    %% Update Dynamic Obstacles
    grid_dynamic = zeros(gridSize);
    for i = 1:length(movingObstacles)
        % Update obstacle position
        movingObstacles(i).pos = movingObstacles(i).pos + movingObstacles(i).vel * params.dt;
        
        % Reflect at boundaries
        if movingObstacles(i).pos(1) < 1 || movingObstacles(i).pos(1) > gridSize(2)
            movingObstacles(i).vel(1) = -movingObstacles(i).vel(1);
            movingObstacles(i).pos(1) = min(max(movingObstacles(i).pos(1),1), gridSize(2));
        end
        if movingObstacles(i).pos(2) < 1 || movingObstacles(i).pos(2) > gridSize(1)
            movingObstacles(i).vel(2) = -movingObstacles(i).vel(2);
            movingObstacles(i).pos(2) = min(max(movingObstacles(i).pos(2),1), gridSize(1));
        end
        
        % Mark the dynamic obstacle on grid_dynamic (rounding to nearest cell)
        row = round(movingObstacles(i).pos(2));
        col = round(movingObstacles(i).pos(1));
        grid_dynamic(row, col) = 1;
    end
    
    % Combine static maze and dynamic obstacles to form the planning grid.
    grid_combined = max(grid_static, grid_dynamic);
    
    %% Determine the Current Goal for DWA
    if global_mode
        % Use current subgoal from the global path. Convert grid cell [row, col]
        % to continuous coordinates: assume cell center is at (col, row)
        current_goal = global_path(subgoal_index, :);
        current_goal = [current_goal(2), current_goal(1)];
    else
        current_goal = main_goal;
    end

    %% Compute Local Control using DWA on the Combined Grid
    [v, omega] = dwa(state, grid_combined, current_goal, params);
    
    %% Check for Progress Toward the Main Goal
    current_distance = norm(state(1:2) - main_goal);
    if abs(last_distance - current_distance) < progress_threshold
        stuck_counter = stuck_counter + 1;
    else
        stuck_counter = 0;
    end
    last_distance = current_distance;
    
    %% Invoke Global Planning if Stuck and Not Already in Global Mode
    if stuck_counter > stuck_limit && ~global_mode
        % Convert current continuous position to grid cell indices (rounding)
        start_cell = [round(state(2)), round(state(1))]; % [row, col]
        goal_cell = [main_goal(2), main_goal(1)];
        global_path = globalPlanner(grid_combined, start_cell, goal_cell);
        if ~isempty(global_path)
            global_mode = true;
            subgoal_index = 1;
            global_mode_iter_count = 0;
            disp('Global planner activated!');
        else
            disp('Global planner failed: no path found.');
        end
        stuck_counter = 0;
    end
    
    %% Global Mode: Check if Subgoal is Reached & Timeout
    if global_mode
        global_mode_iter_count = global_mode_iter_count + 1;
        % Compare continuous robot position with subgoal (already converted)
        if norm(state(1:2) - current_goal) < subgoal_threshold
            subgoal_index = subgoal_index + 1;
            if subgoal_index > size(global_path,1)
                global_mode = false;
                disp('Global path completed. Resuming main goal.');
            else
                disp(['Advancing to subgoal ' num2str(subgoal_index)]);
            end
        end
        if global_mode_iter_count > global_mode_timeout
            global_mode = false;
            disp('Global mode timeout. Reverting to local planning.');
        end
    end
    
    %% Update Robot State using Simple Kinematics
    state(1) = state(1) + v * cos(state(3)) * params.dt;
    state(2) = state(2) + v * sin(state(3)) * params.dt;
    state(3) = state(3) + omega * params.dt;
    trajectory = [trajectory; state(1:2)];
    
    %% Simulate LIDAR Sensor for Visualization
    lidar_ranges = simulateLidar(state, grid_combined, lidar.angles, lidar.max_range);
    
    %% Visualization
    clf;
    imagesc(flipud(grid_combined));
    colormap(gray);
    hold on;
    
    % Plot robot (red circle), main goal (green x), and trajectory (blue line)
    plot(state(1), gridSize(1) - state(2) + 1, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    plot(main_goal(1), gridSize(1) - main_goal(2) + 1, 'gx', 'MarkerSize', 10, 'LineWidth', 2);
    plot(trajectory(:,1), gridSize(1) - trajectory(:,2) + 1, 'b-', 'LineWidth', 2);
    
    % Plot global path (if active) in yellow dashed line
    if global_mode && ~isempty(global_path)
        global_path_xy = [global_path(:,2), global_path(:,1)];
        global_path_xy(:,2) = gridSize(1) - global_path_xy(:,2) + 1;
        plot(global_path_xy(:,1), global_path_xy(:,2), 'y--', 'LineWidth', 2);
    end
    
    % Overlay LIDAR rays in red
    for i = 1:length(lidar.angles)
        angle = state(3) + lidar.angles(i);
        endPt = state(1:2) + lidar_ranges(i) * [cos(angle), sin(angle)];
        plot([state(1) endPt(1)], [gridSize(1)-state(2)+1 gridSize(1)-endPt(2)+1], 'r-');
    end
    
    % Optionally: plot dynamic obstacles as blue squares
    for i = 1:length(movingObstacles)
        pos = movingObstacles(i).pos;
        plot(pos(1), gridSize(1)-pos(2)+1, 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    end
    
    axis([0.5 gridSize(2)+0.5 0.5 gridSize(1)+0.5]);
    axis equal;
    title(sprintf('Iteration: %d', iter));
    drawnow;
    
    % Check if main goal is reached
    if norm(state(1:2) - main_goal) < 0.5
        disp('Main goal reached!');
        break;
    end
    
    pause(0.05);
end

%% --- Helper Functions ---

% DWA: Evaluate candidate control inputs and select the best one
function [best_v, best_omega] = dwa(state, grid, goal, params)
    best_cost = Inf;
    best_v = 0;
    best_omega = 0;
    for v = 0:params.velocity_resolution:params.max_v
        for omega = -params.max_omega:params.omega_resolution:params.max_omega
            cost = evaluateTrajectory(state, v, omega, grid, goal, params);
            if cost < best_cost
                best_cost = cost;
                best_v = v;
                best_omega = omega;
            end
        end
    end
end

% Evaluate a candidate trajectory and return its cost
function cost = evaluateTrajectory(state, v, omega, grid, goal, params)
    t = 0:params.dt:params.predict_time;
    traj = zeros(length(t), 3);
    traj(1,:) = state;
    collision = false;
    for i = 2:length(t)
        traj(i,1) = traj(i-1,1) + v * cos(traj(i-1,3)) * params.dt;
        traj(i,2) = traj(i-1,2) + v * sin(traj(i-1,3)) * params.dt;
        traj(i,3) = traj(i-1,3) + omega * params.dt;
        if checkCollision(traj(i,1:2), grid, params.robot_radius)
            collision = true;
            break;
        end
    end
    if collision
        cost = Inf;
        return;
    end
    goal_cost = norm(traj(end,1:2) - goal);
    desired_heading = atan2(goal(2) - traj(end,2), goal(1) - traj(end,1));
    heading_cost = abs(angdiff(traj(end,3), desired_heading));
    velocity_cost = (params.max_v - v);
    
    safe_distance = 1.0;
    min_clearance = Inf;
    for i = 1:size(traj,1)
        clearance = computeClearance(traj(i,1:2), grid);
        if clearance < min_clearance
            min_clearance = clearance;
        end
    end
    if min_clearance < safe_distance
        clearance_cost = (safe_distance - min_clearance);
    else
        clearance_cost = 0;
    end
    
    w_goal = 0.1;
    w_heading = 0.02;
    w_vel = 0.3;
    w_clearance = 0.8;
    cost = w_goal * goal_cost + w_heading * heading_cost + w_vel * velocity_cost + w_clearance * clearance_cost;
end

% Compute clearance from a point to the nearest obstacle
function clearance = computeClearance(pos, grid)
    [obs_rows, obs_cols] = find(grid == 1);
    if isempty(obs_rows)
        clearance = Inf;
        return;
    end
    dists = sqrt((obs_cols - pos(1)).^2 + (obs_rows - pos(2)).^2);
    clearance = min(dists) - 0.5;
end

% Check collision for a point with given robot_radius
function collision = checkCollision(pos, grid, robot_radius)
    [rows, cols] = size(grid);
    x = pos(1); y = pos(2);
    if (x < 1) || (x > cols) || (y < 1) || (y > rows)
        collision = true;
        return;
    end
    [obs_rows, obs_cols] = find(grid == 1);
    collision = false;
    for k = 1:length(obs_cols)
        cell_center = [obs_cols(k), obs_rows(k)];
        if norm(pos - cell_center) <= (robot_radius + 0.5)
            collision = true;
            return;
        end
    end
end

% Simulate LIDAR sensor by casting rays from the robot position
function distances = simulateLidar(state, grid, angles, max_range)
    pos = state(1:2);
    distances = zeros(size(angles));
    delta = 0.1;
    for i = 1:length(angles)
        angle = state(3) + angles(i);
        distance = 0;
        hit = false;
        while ~hit && distance < max_range
            distance = distance + delta;
            test_point = pos + distance * [cos(angle), sin(angle)];
            if checkCollision(test_point, grid, 0)
                hit = true;
            end
        end
        distances(i) = distance;
    end
end

% Global Planner using A* algorithm on the grid
function path = globalPlanner(grid, start_cell, goal_cell)
    [rows, cols] = size(grid);
    openSet = [];
    closedSet = zeros(rows, cols);
    parent = cell(rows, cols);
    h = @(r,c) sqrt((r - goal_cell(1))^2 + (c - goal_cell(2))^2);
    start_node = struct('row', start_cell(1), 'col', start_cell(2), 'g', 0, 'f', h(start_cell(1), start_cell(2)));
    openSet = [start_node];
    neighbors = [ -1, -1; -1, 0; -1, 1; 0, -1; 0, 1; 1, -1; 1, 0; 1, 1];
    found = false;
    while ~isempty(openSet)
        [~, idx] = min([openSet.f]);
        current = openSet(idx);
        openSet(idx) = [];
        closedSet(current.row, current.col) = 1;
        if current.row == goal_cell(1) && current.col == goal_cell(2)
            found = true;
            break;
        end
        for i = 1:size(neighbors,1)
            r = current.row + neighbors(i,1);
            c = current.col + neighbors(i,2);
            if r < 1 || r > rows || c < 1 || c > cols, continue; end
            if grid(r,c) == 1 || closedSet(r,c) == 1, continue; end
            tentative_g = current.g + norm(neighbors(i,:));
            inOpen = false;
            for j = 1:length(openSet)
                if openSet(j).row == r && openSet(j).col == c
                    inOpen = true;
                    if tentative_g < openSet(j).g
                        openSet(j).g = tentative_g;
                        openSet(j).f = tentative_g + h(r,c);
                        parent{r,c} = [current.row, current.col];
                    end
                    break;
                end
            end
            if ~inOpen
                neighbor_node = struct('row', r, 'col', c, 'g', tentative_g, 'f', tentative_g + h(r,c));
                openSet = [openSet, neighbor_node];
                parent{r,c} = [current.row, current.col];
            end
        end
    end
    if found
        path = [];
        r = goal_cell(1);
        c = goal_cell(2);
        path = [r, c];
        while ~(r == start_cell(1) && c == start_cell(2))
            prc = parent{r, c};
            r = prc(1);
            c = prc(2);
            path = [ [r, c] ; path ];
        end
    else
        path = [];
    end
end

% Helper functions to add or remove obstacles
function grid = addObstacle(grid, cell)
    grid(cell(2), cell(1)) = 1;
end

function grid = removeObstacle(grid, cell)
    grid(cell(2), cell(1)) = 0;
end
