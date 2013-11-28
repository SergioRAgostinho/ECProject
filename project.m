clear all;
clc;

%% Offline Computation
%grid alteral size
gridLateral = 100;

%random occupancy
stateSpace = rand(gridLateral)<0.05;


%obstacles
idObst = find(stateSpace);

%Free spots
freeStates = find(~stateSpace);
nFreeStates = numel(freeStates);

%initial assumptions 4 direction moving and sensing
numSensedDirections=4;

% Pre-alocation of sparse matrix
A = spalloc (numel(freeStates),numel(freeStates),numel(freeStates)*4);
B = spalloc(numel(freeStates),2^(numSensedDirections),numel(freeStates)); %Depending on number of directions measured

%Compute reduced form of B
for iStates = 1:numel(freeStates)
    
    %pre process each position
    pos = freeStates(iStates);
%     pos=iStates
    
    %adjacent positions in linear indexing
    Adj = [pos + gridLateral; pos-1; pos - gridLateral; pos+1];
    
    % sonar's response
    sensArray = ~([ Adj(1) <= gridLateral^2;
                    rem(Adj(2),gridLateral) ~= 0;
                    Adj(3) > 0; 
                    rem(Adj(4),gridLateral) ~= 1] & ... %borders
                    ~ismember(Adj,idObst)); %obstacles

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Create Matrix B
    B(iStates, bi2de(sensArray')+1)=1;
    
    
    
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Create Matrix A
    
%     ~sensArray./sum(~sensArray)
  
    for jPsbStates=1: numSensedDirections
        
        if sensArray(jPsbStates)==0;
%             if  Adj(jPsbStates)>numel(freeStates)
%                 pause
%             end
            A(iStates ,  find( Adj(jPsbStates)==freeStates))= 1;
        end
        
    end
    sumFullARow=sum(full(A(iStates,:)));
    
    A(iStates,:)=A(iStates,:) ./ sumFullARow;
    
end

% 
% % Vizualization of matrix B and A
% spy(B); title('Matrix B');pause(0.5); 
% close;
% figure();
% spy(A); title('Matrix A');pause(0.5); 
% close ;
% % rescalledA=(full(A));
% % figure();imshow(rescalledA,[0 1]);pause;
% 

%% Initial state probability and correspodent image generation
Pi = ones(size(freeStates))/numel(freeStates);

imPi = NaN(gridLateral);
% imAlpha = NaN(gridLateral);
imPi(freeStates) = Pi;


%% Online 

% Robot initialization
% pos0 = freeStates(randi(numel(freeStates),1));
pos0 = datasample(freeStates,1);
pos = pos0;

% Initial Filtering
% sensInt =  B(pos == freeStates);
% alpha = (B == sensInt).*Pi;
% Pi = alpha./sum(alpha);

%adjacent positions in linear indexing
Adj = [pos + gridLateral; pos-1; pos - gridLateral; pos+1];

% sonar's response
sensInt = ~([ Adj(1) <= gridLateral^2;rem(Adj(2),gridLateral) ~= 0;
                Adj(3) > 0;rem(Adj(4),gridLateral) ~= 1] & ... %borders
                ~ismember(Adj,idObst)); %obstacles
y_1 = bi2de(sensInt') + 1;

collB=B(:,y_1); % select collum of B that corresponds to sonar meas.
[rows, cols, vals]=find(collB);
D= sparse(rows,rows,vals,numel(collB),numel(collB)); % constr. D as diag(collB)
% sparseD=diag(collB,0);


alpha=D*Pi;
alpha_old = alpha./sum(alpha);
imPi(freeStates) = alpha_old; 

% %the following 4 lines only have debug purpose-----------------------------
% imAlpha(freeStates)= alpha/max(max(alpha));  falpha_oldigure(9);  imshow(imAlpha);
% figure(10); colorStateSpace(:,:,1)=255*uint8(stateSpace); colorStateSpace(:,:,2)=255*uint8(stateSpace); 
% colorStateSpace(:,:,3)=255*uint8(stateSpace);  
% [idy,idx]=ind2sub([gridLateral,gridLateral],pos); colorStateSpace(idy,idx,:)=[255,0,0];
% imshow(colorStateSpace); pause;  
% %--------------------------------------------------------------------------


%% Quick visualization
hFigure = figure(1);
[iy,ix]=ind2sub([gridLateral,gridLateral],[idObst;pos]); 
hImage = image(imPi);
hold on
hPlot = plot(ix(1:end-1),iy(1:end-1),'xg',ix(end),iy(end),'.k');
%rectangle('Position',[0,0,gridLateral+1,gridLateral+1]);
xlim([-1,gridLateral+2])
ylim([-1,gridLateral+2])
hAxis = gca; pause(2);
hold off

set(hAxis,'CLim',[0 1])
set(hImage,'CDataMapping','scaled')


%% Stop if mouse button is pressed
set(hFigure,'ButtonDownFcn','out = true;');
set(gca,'ButtonDownFcn','out = true;');

% initial assumptions 4 direction moving and sensing
out = false;

while ~out
    
    %%%%%%%%%%%%%%%%%%%%
    % Where can you go?
    %%%%%%%%%%%%%%%%%%%%
        
    % elegible positions to move aka opposite of sonar's response
    elegAdj = ~sensInt;
    psbMov = Adj(elegAdj);

    
    % Movement decision
    pos = datasample(psbMov,1);
    
    % draw updated position
    [iy,ix]=ind2sub([gridLateral,gridLateral],pos);
    set(hPlot(2),'XData',ix(end),'YData',iy(end));
    
    %%%%%%%%%%%%%%%%%%%%
    % What can I see?
    %%%%%%%%%%%%%%%%%%%%
    
    % adjacent positions in linear indexing
    Adj = [pos + gridLateral; pos-1; pos - gridLateral; pos+1];
    
    %Get the sensor output intensity
    y_j = find(B(pos==freeStates,:));
    sensInt=de2bi(y_j - 1,4)'; 
    

    % building D
    D = sparse(1:nFreeStates,1:nFreeStates,B(:,y_j), ...
                nFreeStates,nFreeStates);
    
    alpha= D * A' * alpha_old;
        
    alpha_old=alpha/sum(alpha);
    
    %Update images
    imPi(freeStates) = alpha_old.^(1/2);
    
    set(hImage,'CData',imPi);
    
%     for j = find(alpha)
%         
%     end
    
    %%%%%%%%%%%%%%%%%%%%
    % Move
    %%%%%%%%%%%%%%%%%%%


    %%%%%%%%%%%%%%%%%%%%
    % What can 
    %%%%%%%%%%%%%%%%%%%%
    
    pause(.01)
end

close(hFigure)
