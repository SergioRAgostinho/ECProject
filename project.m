%% Offline Computation

%grid alteral size
gridLateral = 10;

%random occupancy
stateSpace = rand(gridLateral)<0.1;

%obstacles
idObst = find(stateSpace);

%Free spots
freeStates = find(~stateSpace);
B = NaN(size(freeStates));

%Compute reduced form of B
for iStates = 1:numel(freeStates)
    
    %initial assumptions 4 direction moving and sensing
    
    %pre process each position
    pos = freeStates(iStates);
    
    %adjacent positions in linear indexing
    Adj = [pos + gridLateral; pos-1; pos - gridLateral; pos+1];
    
    % sonar's response
    sensArray = ~([ Adj(1) <= gridLateral^2;rem(Adj(2),gridLateral) ~= 0;
                    Adj(3) > 0;rem(Adj(4),gridLateral) ~= 1] & ... %borders
                    ~ismember(Adj,idObst)); %obstacles
    B(iStates) = bi2de(sensArray');
end

%% Initial state probability and correspodent image generation
Pi = ones(size(freeStates))/numel(freeStates);

imPi = NaN(gridLateral);
imPi(freeStates) = Pi;


%% Online 

%Robot initialization
% pos0 = freeStates(randi(numel(freeStates),1));
pos0 = datasample(freeStates,1);
pos = pos0;

%Initial Filtering
sensInt =  B(pos == freeStates);
alpha = (B == sensInt).*Pi;
Pi = alpha./sum(alpha);
alpha_old = alpha;
imPi(freeStates) = Pi;



%% Quick visualization
hFigure = figure(1);
[iy,ix]=ind2sub([gridLateral,gridLateral],[idObst;pos]);
hImage = image(imPi);
hold on
hPlot = plot(ix(1:end-1),iy(1:end-1),'xg',ix(end),iy(end),'.k');
%rectangle('Position',[0,0,gridLateral+1,gridLateral+1]);
xlim([-1,gridLateral+2])
ylim([-1,gridLateral+2])
hAxis = gca;
hold off

set(hAxis,'CLim',[0 1])
set(hImage,'CDataMapping','scaled')


%% Stop if mouse button is pressed
set(hFigure,'ButtonDownFcn','out = true;');
set(gca,'ButtonDownFcn','out = true;');

%initial assumptions 4 direction moving and sensing
out = false;

% while ~out
    
    %%%%%%%%%%%%%%%%%%%%
    % Where can you go?
    %%%%%%%%%%%%%%%%%%%%
    
    %adjacent positions in linear indexing
    Adj = [pos + gridLateral; pos-1; pos - gridLateral; pos+1];
    
    %elegible positions to move aka opposite of sonar's response
    elegAdj = ~de2bi(sensInt,4)';
    psbMov = Adj(elegAdj);
    a_i = ones(size(psbMov))./numel(psbMov);
    
    %Movement decision
    pos = datasample(Adj(elegAdj),1);
    
    %draw updated position
    [iy,ix]=ind2sub([gridLateral,gridLateral],pos);
    set(hPlot(2),'XData',ix(end),'YData',iy(end));
    
    %%%%%%%%%%%%%%%%%%%%
    % Where can I be?
    %%%%%%%%%%%%%%%%%%%%
    sensInt =  B(pos == freeStates);
    alpha = (B == sensInt);
    
    for j = find(alpha)
        
    end
    
    %%%%%%%%%%%%%%%%%%%%
    % What can 
    %%%%%%%%%%%%%%%%%%%%
    %pause(0.5)
% end

%close(hFigure)