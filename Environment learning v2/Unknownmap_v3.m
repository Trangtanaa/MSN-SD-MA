%% Unknownmap Localcom V3
% UnKnownmap v2 but have some upgrades in node classification to work
% better in large map
% solve problem of float nodes moving near sink

%%
clc;
clear;

%% 
for TIME=1:50
name = ['./case study robustness/map2 v1.2 robust/map2_', num2str(TIME), '.mat']; 
%load(name);
close all;
%% Network parameter

% Monitor area
Obstacle_Area = genarea();
Covered_Area = zeros(size(Obstacle_Area,1),size(Obstacle_Area,2));
[obs_row, obs_col] = find(Obstacle_Area == 0);

% nodes info
MaxIt = 800;              % Maximum Number of Iterations
a = 1;                    % Acceleration Coefficient Upper Bound
N = 60;
rc = 16;
rs = 8;
sink=[5 5];
trap_thresh = 10;         % trap node condition
float_thresh= 100;        % float node condition

%moving parameter
v=5;                      % max velocity of node

%% Init first pop
figure;
initpop=unifrnd(min(obs_row)+1,10,[1 2*N]);
initpop(1,1:2)=sink;
pop=initpop;

% Node role Counter
no_move_counts = zeros(N,1);
no_profit_move_counts = zeros(N,1);
trap_matrix = zeros(N,1);

% Array to Hold Best Cost Values
BestCostIt = zeros(MaxIt, 1);
popIt=repmat(pop,[MaxIt 1]);

%%     ABC Main Loop
for it = 1:MaxIt
    G=Graph(pop,rc);
    %% Trap matrix update
    al_trap_matrix=zeros(N,1);
    for l=2:N
        K=neighbors(G,l);
        al_trap_matrix(l) = no_move_counts(l) + mean(trap_matrix(K));
    end
    trap_matrix=al_trap_matrix;
    clear al_trap_matrix l K;

    %% Decision order
    % order of nodes decide which one will take move first

    distance = distances(G, 1,'Method','unweighted');
    N_layers = max(distance); % max layers= max distance to start node
    
    nodesInLayers = cell(N_layers, 2);  % batch layer matrix
    orderInLayers = cell(N_layers, 1);  % order layer matrix

    % separate to batchs
    for d = 1 : N_layers
        nodesInLayer = find(distance == d);
        sub_G=subgraph(G,nodesInLayer);
        sub_batchs= conncomp(sub_G);
        for i = 1: max(sub_batchs)
            k = sub_batchs==i;
            nodesInLayers{d,i} = nodesInLayer(k);  % Lưu các node vào cell tương ứng với từng tầng
        end
    end
    % Reorder
    for i = 1 : size(nodesInLayers,1)
        empty_order=[];
        maxLength = max(cellfun(@numel, nodesInLayers(i,:)));
        for k = 1 : maxLength
            for j = 1 : size(nodesInLayers,2)
                if numel(nodesInLayers{i,j}) >= k
                    empty_order=[empty_order nodesInLayers{i,j}(k)];
                end
            end
        end
        orderInLayers{i} = empty_order;
    end
    clear distance N_layers d sub_batchs sub_G i j k nodesInLayer maxLength empty_order nodesInLayers;
    
    %% Decision making
    for Layers = 1 : size(orderInLayers,1)
    for decision = 1:numel(orderInLayers{Layers})
        i = orderInLayers{Layers}(decision);           % node i makes move decision this turn
        al_pop=pop(1,(i*2-1):(i*2));             % alternative array of pop to load new positions
        G=Graph(pop,rc);

        % neighbor sensor group of node i
        K = neighbors(G,i);
        neighbor_pop=[];
        for n=1:numel(K)
            neighbor_pop=[neighbor_pop pop(2*K(n)-1) pop(2*K(n))];
        end

        %% New Positions are created depends on types of node
        while (1)
            phi = a*unifrnd(-1, +1, [1 2])*(1-no_profit_move_counts(i)/MaxIt)^2;
            % --------------------------This node is trap node-----------------
            if no_move_counts(i) > trap_thresh
                [new_neigh_Cov, ~]=Cov_Func([neighbor_pop ]        ,rs,Obstacle_Area,Covered_Area);
                [old_neigh_Cov, ~]=Cov_Func([neighbor_pop pop(1,(i*2-1):(i*2)) ],rs,Obstacle_Area,Covered_Area);
                % trap node can explore more
                if old_neigh_Cov-new_neigh_Cov>0.05*old_neigh_Cov
                    fitness_ratio=1;
                    k = K(randi([1 numel(K)]));
                    al_pop(1,1:2) = pop(1,(i*2-1):(i*2)) + phi.*([v v]);
                    no_profit_move_counts(i)=float_thresh/5;
                % trap node get stuck in local optimum
                else
                    fitness_ratio=0;
                    al_pop(1,1:2) = pop(1,(i*2-1):(i*2)) + phi.*([v v]);
                    no_profit_move_counts(i)=no_profit_move_counts(i)+float_thresh/5;
                    no_move_counts(i)=0;
                end
            % --------------------------This node is float node------------------
            elseif no_profit_move_counts(i) > float_thresh
                if rand() >= 0.2
                    [~, n] = max(trap_matrix(K));
                    k=K(n);                             % k is the node that have the most value of no move counts
                    al_pop(1,1:2) = pop(1,(i*2-1):(i*2)) + abs(phi).*( pop(1,(k*2-1):(k*2)) - pop(1,(i*2-1):(i*2)) )*(v/rc);
                    fitness_ratio=0.993;
                else
                    al_pop(1,1:2) = pop(1,(i*2-1):(i*2)) + phi.*[v v];
                    fitness_ratio=1;
                end
            % --------------------------This node is default node---------------
            else
                fitness_ratio=1;
                k = K(randi([1 numel(K)]));
                al_pop(1,1:2) = pop(1,(i*2-1):(i*2)) + phi.*( pop(1,(i*2-1):(i*2)) - pop(1,(k*2-1):(k*2)) )*(v/rc);
            end
        %% boundary check
            al_pop(1,1) = min(max(al_pop(1,1), min(obs_row)+1),size(Obstacle_Area,1));
            al_pop(1,2) = min(max(al_pop(1,2), min(obs_col)+1),size(Obstacle_Area,2));
            obs_check1=[obs_row obs_col]-round(al_pop);
            obs_check2=abs(obs_check1(:,1))+abs(obs_check1(:,2));
            if any(obs_check2==0)==0
                break;
            end

        end
        %% Comparision of cost function
        neighbor_G=Graph([neighbor_pop al_pop],rc);
        if Connectivity_graph(neighbor_G,[])==1
            [new_neigh_Cov, Covered_Area]=Cov_Func([neighbor_pop al_pop(1,1:2)]        ,rs,Obstacle_Area,Covered_Area);
            [old_neigh_Cov, Covered_Area]=Cov_Func([neighbor_pop pop(1,(i*2-1):(i*2)) ],rs,Obstacle_Area,Covered_Area);
            if (new_neigh_Cov) > (old_neigh_Cov)
                no_move_counts(i)=0;
                no_profit_move_counts(i) = max(no_profit_move_counts(i)-1,0);
                pop(1,(i*2-1):(i*2)) = al_pop(1,1:2);
            elseif (new_neigh_Cov) > (old_neigh_Cov*fitness_ratio)
                no_move_counts(i)=0;
                no_profit_move_counts(i)= no_profit_move_counts(i)+1;
                pop(1,(i*2-1):(i*2)) = al_pop(1,1:2);
            else
                no_move_counts(i) = no_move_counts(i)+1;
            end
        else
            no_move_counts(i) = no_move_counts(i)+1;
        end

    end
    end
    clear i j k K n al_pop neighbor_pop neighbor_G phi decisions_order decision new_neigh_Cov old_neigh_Cov fitness_ratio obs_check2 obs_check1;
    
    % Store Best Cost in that iteration
    [BestCostIt(it), Covered_Area] = Cov_Func(pop,rs,Obstacle_Area,Covered_Area);
    popIt(it,:)= pop;
    %disp([num2str(BestCostIt(it)) '  at iteration:  '  num2str(it)]);

    %% plot
    % save figủe
    %{
    save_fig=0;
    if it ==1
        pop=initpop;
        save_fig=1;
    elseif mod(it,100)==0
        save_fig=1;
    end
    %}
    clf();
    hold on;


    %{
    for i = 1:1:numel(G.Edges.EndNodes)/2
        plot([pop(G.Edges.EndNodes(i,1)*2-1),pop(G.Edges.EndNodes(i,2)*2-1)],[pop(G.Edges.EndNodes(i,1)*2),pop(G.Edges.EndNodes(i,2)*2)],'Color','blue','linewidth',1);
    end
    %}
    for i = 1:2:numel(pop)
        plot (pop(1,i) , pop(1,i+1),'ro','MarkerSize', 3,'Color','blue');
        hold on;
        %if save_fig==1 && it~=1
        %    viscircles ([pop(1,i) pop(1,i+1)],rs,'LineWidth',1,'Color', 'k');
        %    text (pop(1,i) , pop(1,i+1), num2str(i/2+0.5),'FontSize',15,'Color','red');
        %end
    end
    clear i;

    % show map
    plot(obs_row-1, obs_col-1,'.', 'MarkerSize', 20, 'Color', 'green');
    %[obs_row, obs_col] = find(Obstacle_Area == 1);
    %plot(obs_row, obs_col,'.', 'MarkerSize', 20, 'Color', 'red');
    [discovered_obs_row, discovered_obs_col] = find(Covered_Area == -1);                    % show discovered map
    plot(discovered_obs_row-1, discovered_obs_col-1,'.', 'MarkerSize', 20, 'Color', 'red');

    xlim([0 size(Obstacle_Area,1)]);
    ylim([0 size(Obstacle_Area,2)]);
    title([num2str(BestCostIt(it)*100) '%  at time step:  '  num2str(it)]);
    grid on;
    drawnow;
    %{
    if save_fig==1
        saveas(gcf, ['Iteration' num2str(it) '.pdf' ]); % Lưu dưới dạng PDF
    end
    %}
    %pause(0.05);
    
end
%%
save(name)
end

