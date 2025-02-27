function G=Graph(pop,rc)
N=numel(pop)/2;
adj_pop=zeros(N);
for i=1:N
    for j=1:N
        dist = sqrt((pop(i*2)-pop(j*2))^2+(pop(i*2-1)-pop(j*2-1))^2);
        if dist <= rc && dist ~=0
            adj_pop(i,j)=dist;
        end
    end
end
G=graph(adj_pop);
