%% Multi-Bot Simulation in a Warehouse Environment using DWA with Global Re-planning
clear; clc; close all;

%% Environment Setup - Simpler Warehouse Layout with Many Rows
gridSize = [20,20];               % Grid dimensions
grid_static = zeros(gridSize);    % 0 = free, 1 = obstacle

% Outer walls
grid_static(1,:) = 1;        % Top wall
grid_static(20,:) = 1;       % Bottom wall
grid_static(:,1) = 1;        % Left wall
grid_static(:,20) = 1;       % Right wall

% Define multiple horizontal shelf rows with wide gaps
% We create three shelf rows at rows 4-5, 9-10, and 14-15.
% In each row, two shelf segments are placed with a wide gap between them.
% Shelf row 1
grid_static(4:5, 3:7)   = 1;    % Shelf segment 1
grid_static(4:5, 12:16) = 1;    % Shelf segment 2

% Shelf row 2
grid_static(9:10, 3:7)   = 1;   % Shelf segment 3
grid_static(9:10, 12:16) = 1;   % Shelf segment 4

% Shelf row 3
grid_static(14:15, 3:7)   = 1;   % Shelf segment 5
grid_static(14:15, 12:16) = 1;   % Shelf segment 6

%% Define Dynamic (Moving) Obstacles (Optional)
movingObstacles(1).pos = [10, 5];
movingObstacles(1).vel = [0.1, 0.05];
movingObstacles(2).pos = [15, 10];
movingObstacles(2).vel = [-0.1, 0.05];
movingObstacles(3).pos = [5, 15];
movingObstacles(3).vel = [0.05, -0.1];

%% Robot and Simulation Parameters
params.max_v = 1.0;              % Maximum linear velocity [m/s]
params.max_omega = 1.0;          % Maximum angular velocity [rad/s]
params.dt = 0.1;                 % Time step [s]
params.predict_time = 3.0;       % Prediction horizon [s]
params.velocity_resolution = 0.1;
params.omega_resolution = 0.1;
params.robot_radius = 0.5;

lidar.angles = -pi/6 : pi/36 : pi/6;
lidar.max_range = 4;

global_mode_timeout = 30;        % Global planning mode timeout (iterations)
subgoal_threshold = 0.7;         % Distance threshold for subgoal attainment
goal_reached_threshold = 0.5;    % Distance to consider the goal reached

%% Multi-Bot Setup (only two bots)
numBots = 2;
bots = struct([]);

% Bot 1: from bottom-left to top-right
bots(1).state = [2, 2, 0];
bots(1).main_goal = [18, 18];
bots(1).trajectory = bots(1).state(1:2);
bots(1).global_mode = false;
bots(1).global_path = [];
bots(1).subgoal_index = 1;
bots(1).global_mode_iter_count = 0;
bots(1).stuck_counter = 0;
bots(1).last_distance = norm(bots(1).state(1:2) - bots(1).main_goal);
bots(1).color = 'r';
bots(1).finished = false;  % New flag: finished moving

% Bot 2: from bottom-right to top-left
bots(2).state = [2, 18, pi/2];
bots(2).main_goal = [16, 2];
bots(2).trajectory = bots(2).state(1:2);
bots(2).global_mode = false;
bots(2).global_path = [];
bots(2).subgoal_index = 1;
bots(2).global_mode_iter_count = 0;
bots(2).stuck_counter = 0;
bots(2).last_distance = norm(bots(2).state(1:2) - bots(2).main_goal);
bots(2).color = 'g';
bots(2).finished = false;  % New flag: finished moving

max_iterations = 1000;

%% Simulation Loop
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
        row = round(movingObstacles(i).pos(2));
        col = round(movingObstacles(i).pos(1));
        grid_dynamic(row, col) = 1;
    end
    % Combined grid: static + dynamic obstacles
    grid_combined_all = max(grid_static, grid_dynamic);
    
    %% Update each bot independently
    for i = 1:numBots
        % If the bot already reached its goal, skip update.
        if bots(i).finished
            continue;
        end
        
        % Check if current bot has reached its main goal
        if norm(bots(i).state(1:2) - bots(i).main_goal) < goal_reached_threshold
            bots(i).finished = true;
            disp(['Bot ' num2str(i) ': Goal reached, stopping.']);
            continue;
        end
        
        % For each bot, add the positions of other bots as obstacles
        grid_bot = grid_combined_all;
        for j = 1:numBots
            if j ~= i
                pos_other = bots(j).state(1:2);
                r = round(pos_other(2));
                c = round(pos_other(1));
                grid_bot(r, c) = 1;
            end
        end
        
        % Determine current goal for local planning
        if bots(i).global_mode
            % Use subgoal from the global plan (convert grid cell to continuous coords)
            current_goal = bots(i).global_path(bots(i).subgoal_index, :);
            current_goal = [current_goal(2), current_goal(1)];
        else
            current_goal = bots(i).main_goal;
        end
        
        % Compute local control using DWA
        [v, omega] = dwa(bots(i).state, grid_bot, current_goal, params);
        
        % Check for progress toward the main goal
        current_distance = norm(bots(i).state(1:2) - bots(i).main_goal);
        if abs(bots(i).last_distance - current_distance) < 0.05
            bots(i).stuck_counter = bots(i).stuck_counter + 1;
        else
            bots(i).stuck_counter = 0;
        end
        bots(i).last_distance = current_distance;
        
        % Invoke global planner if stuck and not already in global mode
        if bots(i).stuck_counter > 15 && ~bots(i).global_mode
            start_cell = [round(bots(i).state(2)), round(bots(i).state(1))];
            goal_cell = [bots(i).main_goal(2), bots(i).main_goal(1)];
            path = globalPlanner(grid_bot, start_cell, goal_cell);
            if ~isempty(path)
                bots(i).global_mode = true;
                bots(i).global_path = path;
                bots(i).subgoal_index = 1;
                bots(i).global_mode_iter_count = 0;
                disp(['Bot ' num2str(i) ': Global planner activated!']);
            else
                disp(['Bot ' num2str(i) ': Global planner failed: no path found.']);
            end
            bots(i).stuck_counter = 0;
        end
        
        % Global mode: check subgoal attainment and timeout
        if bots(i).global_mode
            bots(i).global_mode_iter_count = bots(i).global_mode_iter_count + 1;
            if norm(bots(i).state(1:2) - current_goal) < subgoal_threshold
                bots(i).subgoal_index = bots(i).subgoal_index + 1;
                if bots(i).subgoal_index > size(bots(i).global_path,1)
                    bots(i).global_mode = false;
                    disp(['Bot ' num2str(i) ': Global path completed. Resuming main goal.']);
                else
                    disp(['Bot ' num2str(i) ': Advancing to subgoal ' num2str(bots(i).subgoal_index)]);
                end
            end
            if bots(i).global_mode_iter_count > global_mode_timeout
                bots(i).global_mode = false;
                disp(['Bot ' num2str(i) ': Global mode timeout. Reverting to local planning.']);
            end
        end
        
        % Update bot state using simple kinematics
        bots(i).state(1) = bots(i).state(1) + v * cos(bots(i).state(3)) * params.dt;
        bots(i).state(2) = bots(i).state(2) + v * sin(bots(i).state(3)) * params.dt;
        bots(i).state(3) = bots(i).state(3) + omega * params.dt;
        bots(i).trajectory = [bots(i).trajectory; bots(i).state(1:2)];
    end
    
    %% Visualization
    clf;
    imagesc(flipud(grid_combined_all));
    colormap(gray);
    hold on;
    
    % Plot dynamic obstacles as blue squares
    for i = 1:length(movingObstacles)
        pos = movingObstacles(i).pos;
        plot(pos(1), gridSize(1)-pos(2)+1, 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    end
    
    % Plot each bot, its goal, trajectory, and (if active) global path
    for i = 1:numBots
        % Plot trajectory
        plot(bots(i).trajectory(:,1), gridSize(1) - bots(i).trajectory(:,2) + 1, [bots(i).color '-'], 'LineWidth', 2);
        % Plot current position and goal
        plot(bots(i).state(1), gridSize(1)-bots(i).state(2)+1, [bots(i).color 'o'], 'MarkerSize', 10, 'LineWidth', 2);
        plot(bots(i).main_goal(1), gridSize(1)-bots(i).main_goal(2)+1, [bots(i).color 'x'], 'MarkerSize', 10, 'LineWidth', 2);
        % Plot global path if in global mode
        if bots(i).global_mode && ~isempty(bots(i).global_path)
            gp = bots(i).global_path;
            gp_xy = [gp(:,2), gp(:,1)];
            gp_xy(:,2) = gridSize(1) - gp_xy(:,2) + 1;
            plot(gp_xy(:,1), gp_xy(:,2), 'y--', 'LineWidth', 2);
        end
        % Plot LIDAR rays for visualization
        lidar_ranges = simulateLidar(bots(i).state, grid_combined_all, lidar.angles, lidar.max_range);
        for j = 1:length(lidar.angles)
            angle = bots(i).state(3) + lidar.angles(j);
            endPt = bots(i).state(1:2) + lidar_ranges(j)*[cos(angle), sin(angle)];
            plot([bots(i).state(1) endPt(1)], [gridSize(1)-bots(i).state(2)+1 gridSize(1)-endPt(2)+1], 'r-');
        end
    end
    
    axis([0.5 gridSize(2)+0.5 0.5 gridSize(1)+0.5]);
    axis equal;
    title(sprintf('Iteration: %d', iter));
    drawnow;
    pause(0.05);
end

%% --- Helper Functions ---
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
    desired_heading = atan2(goal(2)-traj(end,2), goal(1)-traj(end,1));
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
    cost = w_goal*goal_cost + w_heading*heading_cost + w_vel*velocity_cost + w_clearance*clearance_cost;
end

function clearance = computeClearance(pos, grid)
    [obs_rows, obs_cols] = find(grid==1);
    if isempty(obs_rows)
        clearance = Inf;
        return;
    end
    dists = sqrt((obs_cols - pos(1)).^2 + (obs_rows - pos(2)).^2);
    clearance = min(dists) - 0.5;
end

function collision = checkCollision(pos, grid, robot_radius)
    [rows, cols] = size(grid);
    x = pos(1); y = pos(2);
    if (x<1)||(x>cols)||(y<1)||(y>rows)
        collision = true;
        return;
    end
    [obs_rows, obs_cols] = find(grid==1);
    collision = false;
    for k = 1:length(obs_cols)
        cell_center = [obs_cols(k), obs_rows(k)];
        if norm(pos - cell_center) <= (robot_radius + 0.5)
            collision = true;
            return;
        end
    end
end

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
            test_point = pos + distance*[cos(angle), sin(angle)];
            if checkCollision(test_point, grid, 0)
                hit = true;
            end
        end
        distances(i) = distance;
    end
end

function path = globalPlanner(grid, start_cell, goal_cell)
    [rows, cols] = size(grid);
    openSet = [];
    closedSet = zeros(rows, cols);
    parent = cell(rows, cols);
    h = @(r,c) sqrt((r-goal_cell(1))^2 + (c-goal_cell(2))^2);
    start_node = struct('row', start_cell(1), 'col', start_cell(2), 'g', 0, 'f', h(start_cell(1), start_cell(2)));
    openSet = [start_node];
    neighbors = [-1,-1; -1,0; -1,1; 0,-1; 0,1; 1,-1; 1,0; 1,1];
    found = false;
    while ~isempty(openSet)
        [~, idx] = min([openSet.f]);
        current = openSet(idx);
        openSet(idx) = [];
        closedSet(current.row, current.col) = 1;
        if current.row==goal_cell(1) && current.col==goal_cell(2)
            found = true;
            break;
        end
        for i = 1:size(neighbors,1)
            r = current.row+neighbors(i,1);
            c = current.col+neighbors(i,2);
            if r<1 || r>rows || c<1 || c>cols, continue; end
            if grid(r,c)==1 || closedSet(r,c)==1, continue; end
            tentative_g = current.g+norm(neighbors(i,:));
            inOpen = false;
            for j = 1:length(openSet)
                if openSet(j).row==r && openSet(j).col==c
                    inOpen = true;
                    if tentative_g < openSet(j).g
                        openSet(j).g = tentative_g;
                        openSet(j).f = tentative_g+h(r,c);
                        parent{r,c} = [current.row, current.col];
                    end
                    break;
                end
            end
            if ~inOpen
                neighbor_node = struct('row', r, 'col', c, 'g', tentative_g, 'f', tentative_g+h(r,c));
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
        while ~(r==start_cell(1) && c==start_cell(2))
            prc = parent{r,c};
            r = prc(1);
            c = prc(2);
            path = [[r, c]; path];
        end
    else
        path = [];
    end
end
