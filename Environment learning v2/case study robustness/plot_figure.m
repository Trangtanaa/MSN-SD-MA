close all;
clear;
clc;

%% Initialization 
x  = 1:50;
x1 = 1:800;

% map1 result
y.map1 = zeros(1, 50);
y.map1it = zeros(50,800);
for i = 1:numel(x)
   name = ['./map1 robust/map1_', num2str(i), '.mat']; 
   load(name, 'BestCostIt');
   y.map1(i) = BestCostIt(800) * 100;
   y.map1it(i,:) = BestCostIt* 100;
end
clear i name BestCostIt;

% map1.2 result
y.map1_2 = zeros(1, 50);
y.map1_2it = zeros(50,800);
for i = 1:numel(x)
   name = ['./map1.2 robust/map1.2_', num2str(i), '.mat']; 
   load(name, 'BestCostIt');
   y.map1_2(i) = BestCostIt(800) * 100;
   y.map1_2it(i,:) = BestCostIt* 100;
end
clear i name BestCostIt;

% map2 result
y.map2 = zeros(1, 50);
y.map2it=zeros(50,800);
for i = 1:numel(x)
   name = ['./map2 v1.2 robust/map2_', num2str(i), '.mat']; 
   load(name, 'BestCostIt');
   y.map2(i) = BestCostIt(800) * 100;
   y.map2it(i,:) = BestCostIt* 100;
end
clear i name BestCostIt;

% map3 result
y.map3 = zeros(1, 50);
y.map3it=zeros(50,800);
for i = 1:numel(x)
   name = ['./map3 robust/map3_', num2str(i), '.mat']; 
   load(name, 'BestCostIt');
   y.map3(i) = BestCostIt(800) * 100;
   y.map3it(i,:) = BestCostIt(1:800)* 100;
end
clear i name BestCostIt;

%% Results

% Covergence 
figure;
hold on;
xlabel('Time step');
ylabel('Coverage rate (%)');
axis([0 800 0 100]);
colors = lines(5);

%plot(x1, y.map1it, 'color', "#0000FF");
%plot(x1, y.map1_2it, 'color', "#00FFFF");
%plot(x1, y.map2it, 'color', "#D95319");
%plot(x1, y.map3it, 'color', "#EDB120");
%plot(x, mean(y.abc), ':*', 'color', "#EDB120");
for j = 5:10:40
    plot(x1, y.map3it(j,:), 'color', colors(round(j/10), :));
end
plot(x1, y.map3it(48,:), 'color', colors(round(5), :));

%legend(arrayfun(@(i) ['Line ' num2str(i)], 1:5, 'UniformOutput', false),'Location', 'best');
legend('first run','second run','third run', 'fourth run', 'fifth run' ,'Location', 'best');
grid on;
%%
% Boxplot
figure;
colors = {[0.4940 0.1840 0.5560] [0 0.4470 0.7410], ...
     [0.8500 0.3250 0.0980] [0.9290 0.6940 0.1250]};

colors = ['y', 'r', 'c', 'b'];
boxplot([(y.map2)' (y.map1)' (y.map3)' (y.map1_2)']);
h = findobj(gca,'Tag','Box');
for j=1:length(h)
    patch(get(h(j),'XData'),get(h(j),'YData'),colors(j),'FaceAlpha',.5);
end
%name = [num2str(x(i)), ' nodes deployment'];
%title(name);
xticklabels({'Scenario 1','Scenario 2', 'Scenario 3', 'Scenario 4'});
ylabel('Coverage rate (%)')