clear all;
clc;

%% Offline Computation
%grid alteral size
gridLateral = 10;

%random occupancy
stateSpace = rand(gridLateral)<0.1;


%obstacles
idObst = find(stateSpace);

%Free spots
freeStates = find(~stateSpace);

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
    % Go through all the possible measuments (this case = 2⁴ = 16 possibilities)
    for kMeasurement = 1: ( 2^(numel(Adj)  )  )
        
        if  (kMeasurement-1)== bi2de(sensArray')
            B(iStates,kMeasurement)=1;
        end
    end
    
    
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Create Matrix A
  
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


% Vizualization of matrix B and A
spy(B); title('Matrix B');pause(0.5); 
close;
figure();
spy(A); title('Matrix A');pause(0.5); 
close ;
% rescalledA=(full(A));
% figure();imshow(rescalledA,[0 1]);pause;


%% Initial state probability and correspodent image generation
Pi = ones(size(freeStates))/numel(freeStates);

imPi = NaN(gridLateral);
imAlpha = NaN(gridLateral);
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
y_1 = bi2de((~([ Adj(1) <= gridLateral^2;rem(Adj(2),gridLateral) ~= 0;
                Adj(3) > 0;rem(Adj(4),gridLateral) ~= 1] & ... %borders
                ~ismember(Adj,idObst)))'); %obstacles
            
collB=B(:,y_1 + 1); % select collum of B that corresponds to sonar meas.
[rows, cols, vals]=find(collB);
D= sparse(rows,rows,vals,numel(collB),numel(collB)); % constr. D as diag(collB)
sparseD=diag(collB,0);


alpha=D*Pi;
alpha_old = alpha;
imPi(freeStates) = Pi; 

%the following 4 lines only have debug purpose-----------------------------
imAlpha(freeStates)= alpha/max(max(alpha));  figure(9);  imshow(imAlpha);
figure(10); colorStateSpace(:,:,1)=255*uint8(stateSpace); colorStateSpace(:,:,2)=255*uint8(stateSpace); 
colorStateSpace(:,:,3)=255*uint8(stateSpace);  
[idy,idx]=ind2sub([gridLateral,gridLateral],pos); colorStateSpace(idy,idx,:)=[255,0,0];
imshow(colorStateSpace); pause;  
%--------------------------------------------------------------------------


%% Quick visualization
hFigure = figure(1);
[iy,ix]=ind2sub([gridLateral,gridLateral],[idObst;pos]); %% Acho que os indices estão trocados
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
    
    % adjacent positions in linear indexing
    Adj = [pos + gridLateral; pos-1; pos - gridLateral; pos+1];
    
    %Get the sensor output intensity
    sensInt=find(B(find(pos==freeStates),:)); 
    
    % elegible positions to move aka opposite of sonar's response

    elegAdj = ~de2bi(sensInt - 1,4)';
    psbMov = Adj(elegAdj);
%     a_i = ones(size(psbMov))./numel(psbMov);
    
    % Movement decision
    pos = datasample(Adj(elegAdj),1);
    
    % draw updated position
    [iy,ix]=ind2sub([gridLateral,gridLateral],pos);
    set(hPlot(2),'XData',ix(end),'YData',iy(end));
    
    %%%%%%%%%%%%%%%%%%%%
    % Where can I be?
    %%%%%%%%%%%%%%%%%%%%
%     sensInt =  B(pos == freeStates)
%     alpha = (B == sensInt);


    % building D
    [rows, cols, vals]=find(B(find(pos==freeStates) ,:));
    
    D= sparse(rows,rows,vals,numel(collB),numel(collB)); % constr. D as diag(collB)
    
    alpha= D * A' * alpha_old;
        
    alpha_old=alpha;
    
%     for j = find(alpha)
%         
%     end
    
    %%%%%%%%%%%%%%%%%%%%
    % Move
    %%%%%%%%%%%%%%%%%%%


    %%%%%%%%%%%%%%%%%%%%
    % What can 
    %%%%%%%%%%%%%%%%%%%%
    pause(0.5)
end

close(hFigure)
