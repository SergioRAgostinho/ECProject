%grid alteral size
gridLateral = 10;

%random occupancy
stateSpace = rand(gridLateral)<0.3;

%obstacles
idObst = find(stateSpace);

%Free spots
freeStates = find(~stateSpace);

%Robot initialization
pos0 = freeStates(ceil(numel(freeStates)*rand(1)));
pos = pos0;

%Quick visualization
hFigure = figure;
[ix,iy]=ind2sub([gridLateral,gridLateral],[idObst;pos]);
hPlot = plot(ix(1:end-1),iy(1:end-1),'xr',ix(end),iy(end),'.k');
rectangle('Position',[0,0,gridLateral+1,gridLateral+1]);
xlim([-1,gridLateral+2])
ylim([-1,gridLateral+2])
set(hFigure,'ButtonDownFcn','out = true;');
set(gca,'ButtonDownFcn','out = true;');

%initial assumptions 4 direction moving and sensing
out = false;

while ~out
    %adjacent positions in linear indexing
    Adj = [pos + gridLateral; pos-1; pos - gridLateral; pos+1];
    
    %elegible positions to move
    elegAdj = [ Adj(1) <= gridLateral^2;rem(Adj(2),gridLateral) ~= 0;
                Adj(3) > 0;rem(Adj(4),gridLateral) ~= 1] & ... %borders
                ~ismember(Adj,idObst); %obstacles
    psbMoves = Adj(elegAdj);
    
    %Movement decision
    pos = psbMoves(ceil(rand(1)*numel(psbMoves)));
    
    [ix,iy]=ind2sub([gridLateral,gridLateral],pos);
%     disp([ix(end),iy(end)])
 
    set(hPlot(2),'XData',ix(end),'YData',iy(end));
    pause(0.5)
end

close(hFigure)