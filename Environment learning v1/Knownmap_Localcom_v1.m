%% Knownmap Localcom V1
% deploy sensors on a known map, with local communication constrain. Node
% communicate with neighbor one to decide where to explore next. Otherwise,
% node's decision order depends on bfs still not apply multi thread to
% simulate decentralized behavior.
%%
clc;
clear;
close all;

%% Problem Definition
% network parameter
VarMax=100;
VarMin=0;
Monitor_Area=[VarMax VarMax];
Obstacle_Area = genarea();
[obs_row, obs_col] = find(Obstacle_Area == 1);

%
N = 55;
rc = 16;
rs = 8;
sink=[5 5];

%moving parameter
v=2;                            % max velocity of node

%% ABC Settings

MaxIt = 600;                % Maximum Number of Iterations
L = round(MaxIt*2/5);       % Abandonment Limit Parameter (Trial Limit)
a = 1/2;                    % Acceleration Coefficient Upper Bound

%% Init first pop
figure;
pop=unifrnd(0,10,[1 2*N]);
pop(1,1:2)=sink;
% Abandonment Counter
no_move_counts = zeros(N,1);
no_profit_move_counts = zeros(N,1);

% Array to Hold Best Cost Values
BestCostIt = zeros(MaxIt, 1);
popIt=repmat(pop,[MaxIt 1]);
%BestFit = zeros(MaxIt,1);

%%     ABC Main Loop
for it = 2:MaxIt
    G=Graph(pop,rc);
    decisions_order=bfsearch(G,1);
    % Recruited Bees
    for decision = 2:numel(decisions_order)
        i = decisions_order(decision);
        al_pop=pop(1,(i*2-1):(i*2));             % alternative array of pop to load new positions
        G=Graph(pop,rc);

        % Choose k randomlyon neighbors of i
        K = neighbors(G,i);
        k = K(randi([1 numel(K)]));

        % neighbor sensor group of i
        neighbor_pop=[];
        for n=1:numel(K)
            neighbor_pop=[neighbor_pop pop(2*K(n)-1) pop(2*K(n))];
        end

        % Define Acceleration Coeff.
        phi = a*unifrnd(-1, +1, [1 2])*(1-no_move_counts(i)/MaxIt)^2;
        
        % New Positions save to al_pop
        %al_pop(1,2) = pop(1,i*2)  + phi*(pop(1,i*2)  -pop(1,k*2)  );
        %al_pop(1,1) = pop(1,i*2-1)+ phi*(pop(1,i*2-1)-pop(1,k*2-1));
        al_pop(1,1:2) = pop(1,(i*2-1):(i*2)) + phi.*( pop(1,(i*2-1):(i*2)) - pop(1,(k*2-1):(k*2)) );

        % Apply Bounds
        al_pop(1,1:2) = min(max(al_pop(1,1:2), VarMin),VarMax);

        % Comparision of cost function
        neighbor_G=Graph([neighbor_pop al_pop],rc);
        if Connectivity_graph(neighbor_G,[])==1
            new_neigh_Cov=Cov_Func([neighbor_pop al_pop(1,1:2)]        ,rs,Obstacle_Area);
            old_neigh_Cov=Cov_Func([neighbor_pop pop(1,(i*2-1):(i*2)) ],rs,Obstacle_Area);
            if (new_neigh_Cov) > (old_neigh_Cov)
                no_move_counts(i)=0;
                pop(1,(i*2-1):(i*2)) = al_pop(1,1:2);
            elseif (new_neigh_Cov) == (old_neigh_Cov)
                no_move_counts(i)=0;
                no_profit_move_counts(i) = no_profit_move_counts(i)+1;
                pop(1,(i*2-1):(i*2)) = al_pop(1,1:2);
            else
                no_move_counts(i) = no_move_counts(i)+1;
            end
        end
    end
    clear i j k K n al_pop neighbor_pop neighbor_G phi decisions_order decision new_neigh_Cov old_neigh_Cov;
    %{
    % Worst selection
        for i = 1:N
            al_pop=[pop(1,1:((i-1)*2)) pop(1,(i+1)*2-1:2*N)];           % calculate cost when iliminate bee i
            F(i)=Cov_Func(al_pop,rs,Obstacle_Area);                     % contribution of bee i in colony           
        end 
        P=F/sum(F);
    % Onlooker Bees
    for m = 1:nOnlooker % run m times but no use m as a matrix counter
       
        % Select Source Site
        %-------------------------------------------------------------------
        n = RouletteWheelSelection(P);  %not run i from 1 to pop but take the worst selection
        %-------------------------------------------------------------------
        al_pop=pop(1,(n*2-1):(n*2));
        
        % Choose k randomly on neighbors of n
        K = neighbors(G,n);
        k = K(randi([1 numel(K)]));
                
        % neighbor sensor group of n
        neighbor_pop=[];
        for j=1:numel(K)
            neighbor_pop=[neighbor_pop pop(1,2*K(j)-1) pop(1,2*K(j))];
        end

        % Define Acceleration Coeff.
        phi = a*unifrnd(-1, +1, 1)*(1-C(i)/MaxIt)^5;
        
        % New Positions save to al_pop
        al_pop(1,2) = pop(1,n*2)+phi*(pop(1,n*2)-pop(1,k*2));
        al_pop(1,1) = pop(1,n*2-1)+phi*(pop(1,n*2-1)-pop(1,k*2-1));

        % Apply Bounds
        al_pop(1,2) = max(al_pop(1,2), VarMin);
        al_pop(1,2) = min(al_pop(1,2), VarMax);
        al_pop(1,1) = max(al_pop(1,1), VarMin);
        al_pop(1,1) = min(al_pop(1,1), VarMax);
        
        % Comparision of cost function and constrain
        neighbor_G=Graph([neighbor_pop al_pop],rc);
        if Connectivity_graph(neighbor_G,[])==1
            if (Cov_Func([neighbor_pop al_pop],rs,Obstacle_Area)) >= (Cov_Func([ neighbor_pop pop(1,(n*2-1):(n*2)) ],rs,Obstacle_Area)) 
                pop(1,(n*2-1):(n*2)) = al_pop;
            else
                C(n) = C(n)+1;
            end
        end
        
    end
    %}
    

    % Store Best Cost in that iteration
    BestCostIt(it) = Cov_Func(pop,rs,Obstacle_Area);
    popIt(it,:)= pop;
    disp([num2str(BestCostIt(it)) '  at iteration:  '  num2str(it)]);
    %% plot
    G=Graph(pop,rc);
    clf();
    hold on;
    for i = 1:1:numel(G.Edges.EndNodes)/2
        plot([pop(G.Edges.EndNodes(i,1)*2-1),pop(G.Edges.EndNodes(i,2)*2-1)],[pop(G.Edges.EndNodes(i,1)*2),pop(G.Edges.EndNodes(i,2)*2)],'Color','blue','linewidth',1);
    end

    for i = 1:2:numel(pop)
        plot (pop(1,i) , pop(1,i+1),'ro');
        viscircles ([pop(1,i) pop(1,i+1)],rs,'Color', 'k');
        text (pop(1,i) , pop(1,i+1), num2str(i/2+0.5),'FontSize',13,'Color','red');     
    end
    

    plot(obs_row, obs_col,'.', 'MarkerSize', 20, 'Color', 'red');
    xlim([0 100])
    ylim([0 100])
    title(['Coverage Ratio: ', num2str(Cov_Func(pop,rs,Obstacle_Area)*100),'%'])
    grid on;
    drawnow;
    pause(0.05);
    
end
%%


