function [coverage, Covered_Area] = Cov_Func(alpop,rs,Obstacle_Area,Covered_Area)
%This is fitness function to cal coverage ratio of a environment with
%random scale

% pop is a Nx2 dim matrix holding position of N nodes
% stat = [rs ; rc]
% rs are the sensing rad of nodes                                                      

% Obstacle Area : obs = 0; interest =1;
% Covered Area: not_covered = 0; int_covered = 1; obs_covered = -1;
%% recover Covered_Area
% turn covered points to uncovered point to recalculate coverage
[obs_row, obs_col] = find(Covered_Area ~= 0);
for i = 1:numel(obs_col)
    Covered_Area(obs_row(i), obs_col(i)) = 0;
end

%% check sensor covered area
% create alternative area
inside_sector = false(size(Covered_Area,1),size(Covered_Area,2));
%% loop through every node
for j=1:(size(alpop,1))
    % j-th node position 
    x0 = alpop(j,1);
    y0 = alpop(j,2);
    rsJ=rs(j);

    % Boundary constraint
    x_ub=min(ceil(x0+rsJ),size(Covered_Area,1));
    x_lb=max(floor(x0-rsJ),1);
    y_ub=min(ceil(y0+rsJ),size(Covered_Area,1));
    y_lb=max(floor(y0-rsJ),1);

    % Local Grid of j-th node
    [X, Y] = meshgrid(linspace(x_lb, x_ub, x_ub-x_lb+1), linspace(y_lb, y_ub, y_ub-y_lb+1)); 
    
    % Distance matrix of local grid
    D = sqrt((X - x0).^2 + (Y - y0).^2);
    
    % In rs condition
    in_circle = D <= rsJ;
    
    % both conditions applied to alternative area
    inside_sector(y_lb:y_ub,x_lb:x_ub) = inside_sector(y_lb:y_ub,x_lb:x_ub) | (in_circle); 
    
end       
%% merge alternative area with Covered_Area
Covered_Area = inside_sector.* Obstacle_Area - inside_sector.*(1-Obstacle_Area);
%clear D Theta in_circle in_angle inside_sector;


%% add obstacle to Covered_Area
% [obs_row, obs_col] = find(Obstacle_Area == 0);
% for i = 1:numel(obs_col)
%     if Covered_Area (obs_row(i), obs_col(i)) == 1
%         Covered_Area(obs_row(i), obs_col(i)) = -2;
%     end
% end

%% Calculate ratio
count1=numel(find(Covered_Area == 1));		                           % count covered points on wanted location  (wanted)
count2=numel(find(Covered_Area == -2));		                           % count covered points on unwanted location (obstacles)
count3=numel(Obstacle_Area)-numel(find(Obstacle_Area ~= 1));           % count total points on wanted location

coverage=((count1-count2)/count3);	                            % function to avoid obstacles
%coverage=(count1/count3);		                                % function to aim on wanted area

