clear all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Sensing: 8 neighb. and perfect
% Movement: 8 neighb. + don't move
% 


%% Offline Computation
%grid alteral size
tic
gridLateral = 10;

%random occupancy
stateSpace = rand(gridLateral)<0.05;
% stateSpace = zeros(gridLateral);
% stateSpace(2:end-1,51) = 1;
% stateSpace(51,2:end-1) = 1;

%obstacles
idObst = find(stateSpace);

%Free spots
freeStates = find(~stateSpace);
nFreeStates = numel(freeStates);

%initial assumptions 4 direction moving and sensing
nSensDir = 8;
nMovDir = 5;

% Pre-alocations of sparse matrix
A = spalloc (nFreeStates,nFreeStates,nFreeStates*nMovDir);
sensMatrix = false(nFreeStates,nSensDir);

%Compute reduced form of B
for iState = 1:numel(freeStates)
    
    %pre process each position
    pos = freeStates(iState);
    
    %adjacent positions in linear indexing
    Adj = [pos + gridLateral; pos + gridLateral - 1;
           pos-1; pos - gridLateral - 1; 
           pos - gridLateral; pos - gridLateral + 1;
           pos + 1; pos + gridLateral + 1];
    
    % sonar's response
    borders = [ Adj(1) <= gridLateral^2;
                rem(Adj(3),gridLateral) ~= 0;
                Adj(5) > 0; 
                rem(Adj(7),gridLateral) ~= 1];
                
    sensArray = ~([borders(1); borders(1) & borders(2);
                   borders(2); borders(2) & borders(3);
                   borders(3); borders(3) & borders(4);
                   borders(4); borders(4) & borders(1)] & ... %borders
                   ~ismember(Adj,idObst)); %obstacles
             
   
    
    sensMatrix(iState,:) = sensArray';

   
    
    
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % Create Matrix A reducao 23 para 6 segs
     % Dá para optimizar mais but...
     augAdj = [Adj; pos];
     elegAdj = [~sensArray; true];
     probAdj = elegAdj/sum(elegAdj);
     
     [~,idx] = ismember(augAdj(elegAdj),freeStates);
     
     A(iState , idx)= probAdj(elegAdj)';
  
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create Matrix B    Melhoria de 2 Segundos
B = sparse(1:nFreeStates,bi2de(sensMatrix)+1,1, ...
    nFreeStates,2^nSensDir);

clear sensMatrix
toc
%% Initial state probability and correspodent image generation
Pi = ones(size(freeStates))/numel(freeStates);

imPi = NaN(gridLateral);
imPi(freeStates) = Pi;

%Animation
filename = 'animation_neigh8_move9.gif';
animation = false;

% Online 

% Robot initialization
pos0 = datasample(freeStates,1);
pos = pos0;

% Quick visualization
hFigure = figure(1);
[iy,ix]=ind2sub([gridLateral,gridLateral],[idObst;pos]); 

hImage = image(imPi);
hold on

hPlot = plot(ix(1:end-1),iy(1:end-1),'sk', ...
             ix(end),iy(end),'.r');
axis([0,gridLateral+1, 0, gridLateral+1])
set(hPlot,'MarkerSize',25*10/gridLateral)
rectangle('Position',[0.5, 0.5, gridLateral, gridLateral])
hAxis = gca; 
hold off

set(hAxis,'CLim',[0 1])
set(hImage,'CDataMapping','scaled')
colormap(flipud(colormap(bone)))


% Stop if mouse button is pressed
set(hFigure,'ButtonDownFcn','out = true;');
set(gca,'ButtonDownFcn','out = true;');

axis equal
colorbar


%% Initial Filtering
% adjacent positions in linear indexing
Adj = [pos + gridLateral; pos + gridLateral - 1;
       pos-1; pos - gridLateral - 1; 
       pos - gridLateral; pos - gridLateral + 1;
       pos + 1; pos + gridLateral + 1];

% sonar's response
% sonar's response
borders = [ Adj(1) <= gridLateral^2;
            rem(Adj(3),gridLateral) ~= 0;
            Adj(5) > 0; 
            rem(Adj(7),gridLateral) ~= 1];
        
sensInt = ~([borders(1); borders(1) & borders(2);
               borders(2); borders(2) & borders(3);
               borders(3); borders(3) & borders(4);
               borders(4); borders(4) & borders(1)] & ... %borders
               ~ismember(Adj,idObst)); %obstacles
           
y_1 = bi2de(sensInt') + 1;

collB=B(:,y_1); % select collum of B that corresponds to sonar meas.
[rows, cols, vals]=find(collB);
D= sparse(rows,rows,vals,numel(collB),numel(collB)); % constr. D as diag(collB)
% sparseD=diag(collB,0);


alpha=D*Pi;
alpha_old = alpha./sum(alpha);
imPi(freeStates) = alpha_old; 

set(hImage,'CData',imPi);


%Image writing
if animation
    drawnow
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);

    imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0.1);
end


out = false;

while ~out
    
    %%%%%%%%%%%%%%%%%%%%
    % Where can you go?
    %%%%%%%%%%%%%%%%%%%%
        
    % elegible positions to move aka opposite of sonar's response
    augAdj = [Adj; pos];
    elegAdj = [~sensInt;true];
    psbMov = augAdj(elegAdj);
    
     

    
    % Movement decision
    pos = datasample(psbMov,1);
    
    % draw updated position
    [iy,ix]=ind2sub([gridLateral,gridLateral],pos);
    set(hPlot(2),'XData',ix(end),'YData',iy(end));
    
    %%%%%%%%%%%%%%%%%%%%
    % What can I see?
    %%%%%%%%%%%%%%%%%%%%
    
    % adjacent positions in linear indexing
    Adj = [pos + gridLateral; pos + gridLateral - 1;
           pos-1; pos - gridLateral - 1; 
           pos - gridLateral; pos - gridLateral + 1;
           pos + 1; pos + gridLateral + 1];
    
    %Get the sensor output intensity
    y_j = find(B(pos==freeStates,:));
    sensInt=de2bi(y_j - 1,nSensDir)'; 
    

    % building D
    D = sparse(1:nFreeStates,1:nFreeStates,B(:,y_j), ...
                nFreeStates,nFreeStates);
    
    alpha= D * A' * alpha_old;
        
    alpha_old=alpha/sum(alpha);
    
    %Update images
    imPi(freeStates) = alpha_old;%.^(1/2);
    set(hImage,'CData',imPi);
    
   
    pause(0.1)
    
    %Animation part
    if animation
        drawnow
        frame = getframe(1);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);

        imwrite(imind,cm,filename,'gif','WriteMode','append');
    end
end

close(hFigure)
