function [tracks,tracks_lost_1,nextId,FlagPred] = trackletsPolyBbox_V2(...
    PolyBboxes_0,tracks,VisiThre,nextId,frame_i,Level )
%   modified from Motion-Based Multiple Object Tracking
%   input ------------------------
%   bbox         detection results
%   tracks       tracklets
%   VisiThre     threshold for lost and delete
%   nextId       ÏÂÒ»¸ötrack ID
%   frame_i       current processing frame id
%   output ----------------------
%   tracks       tracking results
% % -----------------------------

tracks_lost_1=[];
FlagPred=false;
tracks_0=tracks;
PolyBboxes_1=reshape(PolyBboxes_0,4,[],2);  % 4 points * number * 2 xy
PolyBboxes=permute(PolyBboxes_1,[2 1 3]);  % number * 4 points * 2 xy
%% predict new locations of existing tracks
for i = 1:length(tracks)
    polybbox = tracks(i).polybbox;
    polybboxAll = tracks(i).polybboxAll;  % 4*2*N
    if size(polybboxAll,3)>=2
        N=min([size(polybboxAll,3) 5]);
        Weight=(1:N-1)';
        Weight=Weight/sum(Weight);
        Weight=repmat(Weight,1,2);
        
        polybboxAll_1=polybboxAll(:,:,end-N+1:end);
        CenAll=mean(polybboxAll_1,1);
        CenAll=permute(CenAll,[3 2 1]);
        CenDelta=CenAll(2:end,:)-CenAll(1:end-1,:);
        poly_cenDiff=sum(CenDelta.*Weight,1);
        polybbox=polybbox+repmat(poly_cenDiff,4,1);
    end
    
    tracks(i).polybbox = polybbox;
    tracks(i).frame = [tracks(i).frame; frame_i]; 
    tracks(i).polybboxAll(:,:,end+1)=polybbox;
    tracks(i).age = tracks(i).age + 1;  % update age
end
%% assign detections to tracks | computing cost + assignment | first level
tracks_ForAssign=tracks;
bboxes_ForAssign=PolyBboxes;
tracks_ind=1:length(tracks);
bboxes_ind=1:size(PolyBboxes,1);
nTracks = length(tracks_ind);           % number of tracks
nDetections = length(bboxes_ind);       % number of centroids
assignments_T=[];
ThreIoU=[0.8 0.85];        
ThreIoUDiff=[0.15 0.1];  
centroids=squeeze(mean(bboxes_ForAssign(:,[1 3],:),2));  % center of bbox
centroids=reshape(centroids,[],2);
PolyBboxes_1(end+1,:,:)=nan;
polyin = polyshape(reshape(PolyBboxes_1,[],2),'Simplify' ,false);
if ~isempty(PolyBboxes_1)
polyareas=area(polyin,1:numel(centroids)/2);
end
for assign_i=1 % :2    % two round assign is not necessary
    cost = ones(nTracks, nDetections); % tracks * detections
    for detec_i=1:nDetections
        for track_i = 1:nTracks
            polybox_1=tracks(track_i).polybbox;
            polybox_2=squeeze(PolyBboxes(detec_i,:,:));
            polyInter = OverlapPoly(polybox_1,polybox_2);
            if ~isempty(polyInter)
                polyInterarea=area(polyInter);
                cost(track_i,detec_i)=1-polyInterarea/(tracks(track_i).polyarea+...
                    polyareas(detec_i)-polyInterarea);
            end
        end
        % if one detection bbox correspond to multiple track,recalculate cost
        TRCost=cost(:,detec_i);
        TR_ind=find(TRCost<1);
        if length(TR_ind)>=2
            PolyUnion=polyshape(tracks(TR_ind(1)).polybbox);
            for ci=2:length(TR_ind)
                PolyUnion=union(PolyUnion,polyshape(tracks(TR_ind(ci)).polybbox));
            end
            PolyUnionInter=intersect(PolyUnion,polyshape(polybox_2));
            RatioModi=area(PolyUnionInter,1)/polyareas(detec_i);
            TRCost(TR_ind)=TRCost(TR_ind)*(1+RatioModi);
            cost(:,detec_i)=TRCost;
        end
    end
    cost(cost>1)=1;
    
    costOfNonAssignment = 0.7;   % low --> new track | high --> one track to many detection
    unassignedTrackCost = 10;
    unassignedDetectionCost=0.5;
    %     [assignments, unassignedTracks, unassignedDetections] = ...
    %         assignDetectionsToTracks(cost, costOfNonAssignment); % Munkres' version of the Hungarian algorithm
    [assignments, unassignedTracks, unassignedDetections] = ...
        assignDetectionsToTracks(double(cost), unassignedTrackCost,unassignedDetectionCost); % Munkres' version of the Hungarian algorithm
    % %     modifing assignment
    for cj=1:size(assignments,1)
        TR_cost=cost(assignments(cj,1),assignments(cj,2));
        TR_cost_T=[cost(:,assignments(cj,2)); cost(assignments(cj,1),:)'];
        TR_cost_T=sort(TR_cost_T,'ascend');
        if TR_cost>=ThreIoU(assign_i) || (length(TR_cost_T)>=3 && TR_cost_T(3)-TR_cost_T(2)<ThreIoUDiff(assign_i))
            unassignedTracks=[unassignedTracks;assignments(cj,1)];
            unassignedDetections=[unassignedDetections; assignments(cj,2)];
            assignments(cj,:)=0;
        end
    end
    assignments(assignments(:,1)==0,:)=[];  % uint36
    assignments_T=[assignments_T; ...
        reshape(tracks_ind(assignments(:,1)),[],1) reshape(bboxes_ind(assignments(:,2)),[],1)];
    
    tracks_ForAssign=tracks_ForAssign(unassignedTracks);
    bboxes_ForAssign=bboxes_ForAssign(unassignedDetections,:);
    tracks_ind=tracks_ind(unassignedTracks);
    bboxes_ind=bboxes_ind(unassignedDetections);
    nTracks = length(tracks_ind);           % number of tracks
    nDetections = length(bboxes_ind);   % number of centroids
end

assignments=assignments_T;
unassignedTracks=tracks_ind;
unassignedTracks_Tvisi=[tracks(unassignedTracks).TVisiC];
unassignedDetections=bboxes_ind;
Flag_1=~isempty(unassignedTracks);
Flag_2=false;
if ~isempty(unassignedTracks_Tvisi)
    Flag_2=max(unassignedTracks_Tvisi)>VisiThre;
end
if (Flag_1 || Flag_2) && Level==1 
    FlagPred=true;
    tracks=tracks_0;
    return;
end
%%    % updateAssignedTracks 
numAssignedTracks = size(assignments, 1);
for i = 1:numAssignedTracks
    trackIdx = assignments(i, 1);
    detectionIdx = assignments(i, 2);
    centroid = centroids(detectionIdx, :);
    polybbox = squeeze(PolyBboxes(detectionIdx,:, :));
    polyarea = polyareas(detectionIdx);
    correct(tracks(trackIdx).kalmanFilter, centroid); % correct location with detection
    
    tracks(trackIdx).polybbox=polybbox;   % replace bbox with detection
    tracks(trackIdx).polyarea=polyarea;
    tracks(trackIdx).polybboxAll(:,:,end)=polybbox;
    tracks(trackIdx).TVisiC = ...
        tracks(trackIdx).TVisiC + 1;   % Update visibility.
    tracks(trackIdx).ConInvisiC = 0;
end
%%    % updateUnassignedTracks
for i = 1:length(unassignedTracks)
    ind = unassignedTracks(i);
    tracks(ind).ConInvisiC = ...
        tracks(ind).ConInvisiC + 1;
end
%%    %  deleteLostTracks
if ~isempty(tracks)
    lostInds = [tracks(:).ConInvisiC] >= VisiThre;   % consecutive invisible
    if sum(lostInds)>=1
        tracks_lost_1=tracks(lostInds);
        tracks_lost_1([tracks_lost_1(:).TVisiC]<=VisiThre)=[];
        tracks = tracks(~lostInds); % Delete lost tracks.
    end
end

%%  % createNewTracks
if(~isempty(unassignedDetections))  
    centroids_1 = centroids(unassignedDetections, :); % unassigned detection | centroid
    polyareas_1 = polyareas(unassignedDetections); % unassigned detection | area
    polybboxes_1 = PolyBboxes(unassignedDetections,:, :); % unassigned detection | bbox
    unassignedDetectionsL=length(unassignedDetections); % unassigned detection | number
   for ci=1:unassignedDetectionsL
       nextId=nextId+1;
       centroid = centroids_1(ci,:);
       polybbox = squeeze(polybboxes_1(ci,:, :));  % 4 points * 2 xy
       polyarea = polyareas_1(ci, :);
       kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
           centroid, [30, 10], [20, 10], 10);  % Create a Kalman filter object. [30, 5], [20, 5], 20
       newTrack = struct(...
           'frame', uint32(frame_i), ...
           'id', uint32(nextId), ...
           'polybbox', polybbox, ...
           'polyarea', polyarea, ...
           'polybboxAll', polybbox, ...
           'kalmanFilter', kalmanFilter, ...
           'age', uint32(1), ...
           'TVisiC', uint32(1), ...
           'ConInvisiC', uint8(0));  % Create a new track.
       tracks(end + 1) = newTrack;  % Add it to the array of tracks.
   end
end

end

