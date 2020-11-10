function [Low_tracks_out] = TrackletsMergeSingular_11_V1(Low_tracks_in,ThreSeg,ThreM,CharFun)
%  duration criteria: 1)startpoint and endpoint threshould ThreSeg(1);2) duration
%  threshold ThreSeg(2)
%  IoU criteria: 1) the difference between the most and secondly related
%  IoU ThreM(1); 2) the most related IoU > ThreM(2)
%  Low_tracks   tracklets
%  ThreSeg | gap overlap length 
%  ThreMDist | Distance threshold | 2nd/1st(ascend) 1st distance threshold

if length(Low_tracks_in)==1  % pass if only one tracklet
    Low_tracks_out=Low_tracks_in;
    return
end

Low_tracks_out=Low_tracks_in;

TR1=[Low_tracks_in(:).StartEnd]';
Low_tracks_dura=TR1(:,2)-TR1(:,1);

[TracksCostIoU,~] = TrackletsMergeCostM_V1(Low_tracks_in,ThreSeg);

% % assign and delete tracelet
TR_AssignInd=TracksCostIoU;
TR_AssignInd(TR_AssignInd==inf)=0;
TR_AssignInd(TR_AssignInd==1)=0;  % ****************  无overlap
% TR_AssignInd=TR_AssignInd+TR_AssignInd';
TR_AssignInd(TR_AssignInd~=0)=1;
TR_AssignInd_sum=sum(TR_AssignInd);

TR_Assign_ind=find(TR_AssignInd_sum==1);  % only 1 ralated tracelet
TR2_ind_assign_U=[];
TR2_ind_value_U=[];
for ci=1:length(TR_Assign_ind)
    TR1=TR_AssignInd(:,TR_Assign_ind(ci));
    TR1_ind=find(TR1~=0);
    TR2=[TR_Assign_ind(ci) TR1_ind];
    TR2_ind_assign_U=[TR2_ind_assign_U; min(TR2) max(TR2)];
    TR2_ind_value_U=[TR2_ind_value_U TracksCostIoU(min(TR2),max(TR2))];
end

if ~isempty(TR2_ind_assign_U)
[TR2_ind_assign_U,ia,~] = unique(TR2_ind_assign_U,'rows');
TR2_ind_assign_U=TR2_ind_assign_U';
TR2_ind_value_U=TR2_ind_value_U(ia);

% % assign
CostM=sparse(TR2_ind_assign_U(1,:),TR2_ind_assign_U(2,:),TR2_ind_value_U,length(Low_tracks_in),length(Low_tracks_in));
CostM=full(CostM);
CostM(CostM==0)=inf;
costOfNonAssignment = 0.7;   % low --> new track | high --> one track to many detection
unassignedTrackCost = 1;
unassignedDetectionCost=1;
[assignments, ~, ~] = ...
    assignDetectionsToTracks( CostM, unassignedTrackCost,unassignedDetectionCost); % Munkres' version of the Hungarian algorithm
assignments=unique(assignments,'rows');

% %% %     modifing assignment
for cj=1:size(assignments,1)  % IoU 太小不融合
    TR_cost=CostM(assignments(cj,1),assignments(cj,2));
    if TR_cost>=ThreM(2) || min(Low_tracks_dura(assignments(cj,:)))<=ThreSeg(3)
        assignments(cj,:)=0;
    end
end
assignments(assignments(:,1)==0,:)=[];

if ~isempty(assignments)
    for ci=1:size(assignments,1)  % 融合后不能变短
        Ind_1=assignments(ci,1);
        Ind_2=assignments(ci,2);
        frame_1_ind=Low_tracks_in(Ind_1).StartEnd;
        frame_2_ind=Low_tracks_in(Ind_2).StartEnd;
        if frame_1_ind(2)>=frame_2_ind(2)
            assignments(ci,:)=0;
        end
    end
end
assignments(assignments(:,1)==0,:)=[];

% % combine tracks
Low_tracks_out = TrackletsMergeOperation_V1(Low_tracks_in,assignments);
end
end

