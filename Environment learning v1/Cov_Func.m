function coverage = Cov_Func(pop,Rs,Area1)
%This is fitness function to cal random area coverage ratio

%pop is a 1x2Ndim matrix holding position of nodes
%Rs is the sensing rad of nodes
%Area is a 2dim matrix, stands for the rectangle area
%Area1 is a 2dim matrix, stands for the random area

count1=0;		    % count covered points on wanted location  (wanted)
count2=0;		    % count covered points on unwanted location (obstacles)
count3=0;           % count total points on wanted location
pts=[100,100];     	% distribute pts points throughout the map

Obstacle_Area= Area1;                            
Monitor_Area = [sqrt(numel(Area1)) sqrt(numel(Area1))];
for i=0:(pts(1))        % 0->100
    for k=0:(pts(2))    % 0->100
        if Obstacle_Area(i+1,k+1) > 0
            count3=count3+1;
        end
        for j=1:2:numel(pop)
            dist = sqrt((i*Monitor_Area(1)/pts(1)-pop(1,j))^2+(k*Monitor_Area(2)/pts(2)-pop(1,j+1))^2);
            if (dist< Rs || dist== Rs) && Obstacle_Area(i+1,k+1) > 250
                %coverarea(i+1,k+1)=1;
                count1=count1+1;
                break   
            end
            if (dist< Rs || dist== Rs) && Obstacle_Area(i+1,k+1) < 250
                %coverarea(i+1,k+1)=-2;
                count2=count2+1;
                break   
            end
        end
    end    
end

coverage=((count1-count2)/count3);	    % function to avoid obstacles
%coverage=(count1/count3);		    % function to aim on wanted area
