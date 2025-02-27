clc;
clear;
close all;
figure;

%% Problem Definition
% network parameter
VarMax=100;
VarMin=0;
Monitor_Area=[VarMax VarMax];
Obstacle_Area = genarea();
[obs_row, obs_col] = find(Obstacle_Area == 1);
N = 60;
rc=18;
rs=9;
sink=[10 10];

%moving parameter
v=2;                            % max velocity of node

%% ABC Settings

MaxIt = 400;                % Maximum Number of Iterations
nPop = 50;                  % Population Size (Colony Size)
nOnlooker = nPop;         % Number of Onlooker Bees
L = round(MaxIt*2/5);       % Abandonment Limit Parameter (Trial Limit)
a = 1/2;                      % Acceleration Coefficient Upper Bound

%% Init first pop
pop=unifrnd(0,10,[1 2*N]);
BestCost = Cov_Func(pop,rs,Obstacle_Area);

% Abandonment Counter
C = zeros(N,1);

% Array to Hold Best Cost Values
BestCostIt = zeros(MaxIt, 1);
%BestFit = zeros(MaxIt,1);

%%     ABC Main Loop
G_init=Graph(pop,rc);
pop_init=pop;
for it = 1:MaxIt
    
    % Recruited Bees
    for i = 1:N
        % reset al_pop to the current value of pop
        al_pop=pop;             % alternative array of pop to load exploitative solution
        % Choose k randomly, not equal to i
        K = [1:i-1 i+1:N];
        k = K(randi([1 numel(K)]));
        
        % Define Acceleration Coeff.
        phi = a*unifrnd(-1, +1, 1)*(1-C(i)/MaxIt)^5;
        
        % New Bee Position save to al_pop
        al_pop(1,i*2) = pop(1,i*2)+phi*(pop(1,i*2)-pop(1,k*2));
        al_pop(1,i*2-1) = pop(1,i*2-1)+phi*(pop(1,i*2-1)-pop(1,k*2-1));

        % Apply Bounds
        al_pop(1,i*2) = max(al_pop(1,i*2), VarMin);
        al_pop(1,i*2) = min(al_pop(1,i*2), VarMax);
        al_pop(1,i*2-1) = max(al_pop(1,i*2-1), VarMin);
        al_pop(1,i*2-1) = min(al_pop(1,i*2-1), VarMax);

        % Comparision of cost function
        al_G=Graph(al_pop,rc);
        if Connectivity_graph(al_G,[])==1
            if (Cov_Func(al_pop,rs,Obstacle_Area) >= BestCost) 
                pop = al_pop;
                BestCost = Cov_Func(al_pop,rs,Obstacle_Area);
            else
                C(i) = C(i)+1;
            end
        end
    end

    disp([num2str(BestCost) '  and  '  num2str(it)]);
    % Store Best Cost Ever Found
    BestCostIt(it) = BestCost;
    %% plot
    G=Graph(pop,rc);
    clf();
    for i = 1:2:numel(pop)
        plot (pop(1,i) , pop(1,i+1),'ro');
        hold on;
        text (pop(1,i) , pop(1,i+1), num2str(i/2+0.5),'FontSize',15);
        %hold on;
        viscircles ([pop(1,i) pop(1,i+1)],rs,'Color', 'k');
    end
    
    for i = 1:1:numel(G.Edges.EndNodes)/2
        plot([pop(G.Edges.EndNodes(i,1)*2-1),pop(G.Edges.EndNodes(i,2)*2-1)],[pop(G.Edges.EndNodes(i,1)*2),pop(G.Edges.EndNodes(i,2)*2)],'Color','blue','linewidth',1);
    end
    plot(obs_row, obs_col,'.', 'MarkerSize', 20, 'Color', 'red');
    
    xlim([0 100])
    ylim([0 100])
    title(['Coverage Ratio: ', num2str(BestCost*100),'%'])
    grid on;
    drawnow;
    pause(0.05);
    
end



