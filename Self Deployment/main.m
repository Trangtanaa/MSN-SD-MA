%% Unknownmap Localcom V3
% better in large map
% solve problem of float nodes moving near sink
% hetero sensor network

%%
clc;
clear;

%% 
%for TIME=1:50
%name = ['./case study robustness/map2 v1.2 robust/map2_', num2str(TIME), '.mat']; 
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
N = 50;
rc = 16;
rs = 8*ones(1,N);
sink=[10 10];
trap_thresh = 10;         % trap node condition
float_thresh= 100;        % float node condition

%moving parameter
v=5;                      % max velocity of node
safe_d=1;

%% Init first pop
figure;
initpop=unifrnd(max(sink(1)-rc/2,2),sink(2)+rc/2,[N 2]);
initpop(1,1:2)=sink;
pop=initpop;

% Node role Counter
no_move_counts = zeros(N,1);
no_profit_move_counts = zeros(N,1);
trap_matrix = zeros(N,1);

% Array to Hold Best Cost Values
BestCostIt = zeros(MaxIt, 1);
popIt=zeros(MaxIt,2*N);
popIt(1,:)=reshape(pop,[1 N*2]);
%%     ABC Main Loop
for it = 2:MaxIt
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
        al_pop=pop(i,:);             % alternative array of pop to load new positions
        G=Graph(pop,rc);

        % neighbor sensor group of node i
        K = neighbors(G,i);
        neighbor_pop=[];
        for n=1:numel(K)
            neighbor_pop=[neighbor_pop; pop(K(n),:)];
        end

        %% New Positions are created depends on types of node
        while (1)
            phi = a*unifrnd(-1, +1, [1 2])*(1-no_profit_move_counts(i)/MaxIt)^2;
            % --------------------------This node is trap node-----------------
            if no_move_counts(i) > trap_thresh
                [new_neigh_Cov, ~]=Cov_Func([neighbor_pop ]         ,rs,Obstacle_Area,Covered_Area);
                [old_neigh_Cov, ~]=Cov_Func([neighbor_pop; pop(i,:) ],rs,Obstacle_Area,Covered_Area);
                % trap node can explore more (case 1)
                if old_neigh_Cov-new_neigh_Cov>0.05*old_neigh_Cov
                    fitness_ratio=1;
                    k = K(randi([1 numel(K)]));
                    al_pop = pop(i,:) + phi.*([v v]);
                    node_type = 1;
                    
                % trap node get stuck in local optimum (case 2)
                else
                    fitness_ratio=0;
                    al_pop = pop(i,:) + phi.*([v v]);
                    node_type = 2;
                    
                end
            % --------------------------This node is float node------------------
            elseif no_profit_move_counts(i) > float_thresh
                if rand() >= 0.2
                    [~, n] = max(trap_matrix(K));
                    k=K(n);                             % k is the node that have the most value of no move counts
                    al_pop = pop(i,:) + abs(phi).*(pop(k,:) - pop(i,:))*(v/rc);
                    fitness_ratio=0.993;
                    node_type = 3;
                else
                    al_pop = pop(i,:) + phi.*[v v];
                    fitness_ratio=1;
                    node_type = 4;
                end
            % --------------------------This node is default node---------------
            else
                fitness_ratio=1;
                k = K(randi([1 numel(K)]));
                al_pop(1,1:2) = pop(i,:) + phi.*(pop(i,:) - pop(k,:))*(v/rc);
                node_type = 5;
            end
        %% boundary and collision check
            al_pop(1,1) = min(max(al_pop(1,1), min(obs_row)+1),size(Obstacle_Area,1));
            al_pop(1,2) = min(max(al_pop(1,2), min(obs_col)+1),size(Obstacle_Area,2));

            obs = [obs_col, obs_row; pop(1:i-1,:) ; pop(i+1:N,:)];
            obs_check1=sqrt((obs(:,1)-al_pop(1,1)).^2+(obs(:,2)-al_pop(1,2)).^2);
            if all(obs_check1>safe_d)
                break;
            end

        end
        % update node type
        switch node_type
            case 1
                no_profit_move_counts(i)=float_thresh/10;
            case 2
                no_profit_move_counts(i)=no_profit_move_counts(i)+float_thresh/10;
                no_move_counts(i)=0;
        end
        node_type = 0;
        %% Comparision of cost function
        neighbor_G=Graph([neighbor_pop; al_pop],rc);
        if Connectivity_graph(neighbor_G,[])==1
            [new_neigh_Cov, Covered_Area]=Cov_Func([neighbor_pop; al_pop]  ,rs,Obstacle_Area,Covered_Area);
            [old_neigh_Cov, Covered_Area]=Cov_Func([neighbor_pop; pop(i,:)],rs,Obstacle_Area,Covered_Area);
            if (new_neigh_Cov) > (old_neigh_Cov)
                no_move_counts(i)=0;
                no_profit_move_counts(i) = max(no_profit_move_counts(i)-1,0);
                pop(i,:) = al_pop;
            elseif (new_neigh_Cov) > (old_neigh_Cov*fitness_ratio)
                no_move_counts(i)=0;
                no_profit_move_counts(i)= no_profit_move_counts(i)+1;
                pop(i,:) = al_pop;
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
    popIt(it,:)= reshape(pop,[1 N*2]);
    %disp([num2str(BestCostIt(it)) '  at iteration:  '  num2str(it)]);

    %% plot
    % save figure
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
    % show map
    [obs_row, obs_col] = find(Obstacle_Area == 1); % show interest area
    plot(obs_row, obs_col,'.', 'MarkerSize', 0.1, 'Color', 'blue');
    [obs_row, obs_col] = find(Obstacle_Area == 0); % show obs area
    plot(obs_row, obs_col,'.', 'MarkerSize', 8, 'Color', 'black');
    [discovered_obs_row, discovered_obs_col] = find(Covered_Area == -1); % show discovered map
    plot(discovered_obs_row, discovered_obs_col,'.', 'MarkerSize', 5, 'Color', 'red');
    [discovered_row, discovered_col] = find(Covered_Area == 1); % show discovered map
    plot(discovered_row, discovered_col,'.', 'MarkerSize', 5, 'Color', 'green');
    
    G=Graph(pop,rc);
    %for i = 1:1:numel(G.Edges.EndNodes)/2
    %    plot([pop(G.Edges.EndNodes(i,1)*2-1),pop(G.Edges.EndNodes(i,2)*2-1)],[pop(G.Edges.EndNodes(i,1)*2),pop(G.Edges.EndNodes(i,2)*2)],'Color','blue','linewidth',1);
    %end
    theta = linspace(0, 2*pi, 500);
    for i = 1:N
        plot (pop(i,2) , pop(i,1),'ro','MarkerSize', 3,'Color','blue');
        hold on;
        %if save_fig==1 && it~=1
        %viscircles ([pop(i,2) , pop(i,1)],rs(i),'LineWidth',1,'Color', 'blue');
        text (pop(i,2) , pop(i,1), num2str(i),'FontSize',15,'Color','red');
        %end
        x = pop(i,2) + rs(i) * cos(theta);
        y = pop(i,1) + rs(i) * sin(theta);
        fill(x, y, [0.6 1 0.6], 'FaceAlpha', 0.2, 'EdgeColor', 'k');
    end
    clear i;

    xlim([0 size(Obstacle_Area,1)]);
    ylim([0 size(Obstacle_Area,2)]);
    title([num2str(BestCostIt(it)*100) '%  at time step:  '  num2str(it)]);
    %title(['interest points covered: ' num2str(numel(discovered_col)) ', obstacle points covered: ' num2str(numel(discovered_obs_col))])
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
%save(name)
%end

