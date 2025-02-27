function [coverage , Covered_Area] = Cov_Func(pop,Rs,Area1,Covered_Area)
%This is fitness function to cal random area coverage ratio

%pop is a 1x2Ndim matrix holding position of nodes
%Rs is the sensing rad of nodes
%Area1 is a 2dim matrix, stands for the random area

Obstacle_Area= Area1;                            
%Covered_Area = zeros(size(Area1,1),size(Area1,2));

%% recover sensor covered area
[obs_row, obs_col] = find(Covered_Area == 1);
for i = 1:numel(obs_col)
    Covered_Area(obs_row(i), obs_col(i)) = 0;
end

%% check sensor covered area
for j=1:2:numel(pop)
    % y value of sensor is pop(1,j)
    % x value of sensor is pop(1,j+1)
    start_point=[floor(pop(1,j)) floor(pop(1,j+1))];
    for i = 0:(Rs+1)
        for k = 0:(Rs+1)
            map_x= start_point(2)+1;
            map_y= start_point(1)+1;
            dist = sqrt((map_y-pop(1,j)+i)^2+(map_x-pop(1,j+1)+k)^2);
            % case dist <= Rs
            if (dist < Rs || dist== Rs)
                if map_y+i <= size(Area1,1) && map_x+k <= size(Area1,2)  % 2 quarter
                    Covered_Area(map_y+i,map_x+k) = 1;
                end
                if map_y+i <= size(Area1,1) && map_x-k >0                % 3 quarter
                    Covered_Area(map_y+i,map_x-k) = 1;
                end
                if map_y-i > 0 && map_x+k <= size(Area1,2)               % 1 quarter
                    Covered_Area(map_y-i,map_x+k) = 1;
                end
                if map_y-i > 0 && map_x-k > 0                            % 4 quarter
                    Covered_Area(map_y-i,map_x-k) = 1;
                end
            end
        end
    end
end

%% add obstacle to covered area
[obs_row, obs_col] = find(Obstacle_Area == 0);
for i = 1:numel(obs_col)
    if Covered_Area (obs_row(i), obs_col(i)) == 1
        Covered_Area(obs_row(i), obs_col(i)) = -2;
    end
end

count1=numel(find(Covered_Area == 1));		                           % count covered points on wanted location  (wanted)
count2=numel(find(Covered_Area == -2));		                           % count covered points on unwanted location (obstacles)
count3=numel(Obstacle_Area)-numel(find(Obstacle_Area == 0));           % count total points on wanted location
coverage=((count1-count2)/count3);	    % function to avoid obstacles
%coverage=(count1/count3);		        % function to aim on wanted area

%% recover obs covered area

[obs_row, obs_col] = find(Covered_Area == -2);
for i = 1:numel(obs_col)
    Covered_Area(obs_row(i), obs_col(i)) = -1;
end
