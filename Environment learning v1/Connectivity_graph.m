function check = Connectivity_graph(G,bat_ex)
numberNodes=numnodes(G)-numel(bat_ex);
v = dfsearch(G,1);
if (size(v,1)==numberNodes)
    check = 1;                  %connected
else
    check = 0;                  % not connected
end