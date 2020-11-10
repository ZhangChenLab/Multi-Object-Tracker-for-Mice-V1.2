function [Low_tracks_out] = TrackletsMergePredictionDist_V2(Low_tracks_in,ThreSeg,ThreMDist,CharFun)
%  duration criteria: 1)startpoint and endpoint threshould ThreSeg(1);2) duration
%  threshold ThreSeg(2)
%  IoU criteria: 1) the difference between the most and secondly related
%  IoU ThreM(1); 2) the most related IoU > ThreM(2)
%  Low_tracks   tracklets
%  ThreSeg | gap overlap length 
%  ThreMDist | Distance threshold | 2nd/1st(ascend) 1st distance threshold

global TracePredNet

if length(Low_tracks_in)==1  % 只有一段轨迹，不做后续处理
    Low_tracks_out=Low_tracks_in;
    return
end

% tracks trace prediction
TracksTracePred=[];
for ci=1:length(Low_tracks_in)
    TracksTracePred(ci).StartEnd=Low_tracks_in(ci).StartEnd;
    AXY=mean(Low_tracks_in(ci).polybbox,1);
    AXY=permute(AXY,[3 2 1]);
    if size(AXY,1)>=24
        XYPred_T{1,1}=AXY(end-23:end,:);   % forward
        XYPred_T{1,2}=flip(AXY(1:24,:),1); % backward
        for pi=1:2
            XYPred=[];
            XYPred0=XYPred_T{1,pi};
            XYPred(:,1,:)=XYPred0(:,1,:)-TracePredNet.XYDataMean(1);
            XYPred(:,2,:)=XYPred0(:,2,:)-TracePredNet.XYDataMean(2);
            XYPred(:,1,:)=XYPred(:,1,:)/TracePredNet.XYDataLen(1);
            XYPred(:,2,:)=XYPred(:,2,:)/TracePredNet.XYDataLen(2);
            X0=XYPred;
            for ri=1  :6
                if ri==1
                    Test_X=reshape(X0,24*2,[]);
                    Test_Y=TracePredNet.net(Test_X);
                    Test_Y=permute(Test_Y,[3 1 2]);
                    
                    Test_YPred=Test_Y;
                    X0=cat(1,X0,Test_Y);
                    X0(1,:,:)=[];
                else
                    Test_X=reshape(X0,24*2,[]);
                    Test_Y=TracePredNet.net(Test_X);
                    Test_Y=permute(Test_Y,[3 1 2]);
                    
                    Test_YPred=cat(1,Test_YPred,Test_Y);
                    X0=cat(1,X0,Test_Y);
                    X0(1,:,:)=[];
                end
            end
            XYPredNet=Test_YPred;
            XYPredNet(:,1,:)=XYPredNet(:,1,:)*TracePredNet.XYDataLen(1);
            XYPredNet(:,2,:)=XYPredNet(:,2,:)*TracePredNet.XYDataLen(2);
            XYPredNet(:,1,:)=XYPredNet(:,1,:)+TracePredNet.XYDataMean(1);
            XYPredNet(:,2,:)=XYPredNet(:,2,:)+TracePredNet.XYDataMean(2);
            XYPred_T{2,pi}=XYPredNet;
        end
        TracksTracePred(ci).StartEnd(2)=TracksTracePred(ci).StartEnd(2)+6;
        AXY=cat(1,AXY,XYPred_T{2,1});
        TRStart=double(TracksTracePred(ci).StartEnd(1));
        if TRStart>=7
            TracksTracePred(ci).StartEnd(1)=TracksTracePred(ci).StartEnd(1)-6;
            AXY=cat(1,flip(XYPred_T{2,2},1),AXY);
        elseif TRStart>=2
            TracksTracePred(ci).StartEnd(1)=1;
            AXY=cat(1,flip(XYPred_T{2,2}(1:TRStart-1,:),1),AXY);
        end
        
%         figure; hold on
%         plot(AXY(:,1),AXY(:,2),'k')
%         plot(XYPred_T{2,1}(:,1),XYPred_T{2,1}(:,2),'r')
%         plot(XYPred_T{2,2}(:,1),XYPred_T{2,2}(:,2),'b')
    end
    TracksTracePred(ci).TraceXY=AXY;
end

TracksCostDist = TrackletsTraceMergeCostM_V1(TracksTracePred);

% %% cost matrix
TR_cost=TracksCostDist;
% TR_cost(TR_cost==inf)=1;
% TR_cost=TR_cost+TR_cost'-1;
[TR_cost_2,TR_cost_ind]=sort(TR_cost,'ascend');

TR2_0=TR_cost_2(2,:)./TR_cost_2(1,:);
TR2_0(isnan(TR2_0))=0;
TR2_1=TR2_0>=ThreMDist(1);
TR2_0(TR2_0==inf)=2.5;
TR2_2=TR_cost_2(1,:)<=ThreMDist(2)*(1+TR2_0);  % 动态阈值
TR2_3=TR_cost_2(2,:)>=ThreMDist(3);
TR3=TR2_1.*TR2_2.*TR2_3;
TR2_ind=find(TR3==1);

TR2_ind_1=TR_cost_ind(1,TR2_ind); % 
TR2_ind_value=TR_cost_2(1,TR2_ind);
TR2_ind_assign=[TR2_ind; TR2_ind_1];
TR2_ind_assign=sort(TR2_ind_assign,'ascend');
[TR2_ind_assign_U,ia,~] = unique(TR2_ind_assign','rows');
TR2_ind_assign_U=TR2_ind_assign_U';
TR2_ind_value_U=TR2_ind_value(ia);
% % assign
CostM=sparse(TR2_ind_assign_U(1,:),TR2_ind_assign_U(2,:),TR2_ind_value_U,length(Low_tracks_in),length(Low_tracks_in));
CostM=full(CostM);
CostM=CostM/ThreMDist(2);
CostM(CostM==0)=inf;
costOfNonAssignment = 0.7;   % low --> new track | high --> one track to many detection
unassignedTrackCost = 1;
unassignedDetectionCost=1;
[assignments, ~, ~] = ...
    assignDetectionsToTracks( CostM, unassignedTrackCost,unassignedDetectionCost); % Munkres' version of the Hungarian algorithm
assignments=unique(assignments,'rows');

% %% %     modifing assignment
TR1=[Low_tracks_in(:).StartEnd]';
Low_tracks_dura=TR1(:,2)-TR1(:,1);

% for cj=1:size(assignments,1)  % 
%     if min(Low_tracks_dura(assignments(cj,:)))<=ThreSeg(3)
%         assignments(cj,:)=0;
%     end
% end
assignments(assignments(:,1)==0,:)=[];
if ~isempty(assignments)
    for ci=1:size(assignments,1)  % 
        Ind_1=assignments(ci,1);
        Ind_2=assignments(ci,2);
        f_1_ind=double(Low_tracks_in(Ind_1).StartEnd);
        f_2_ind=double(Low_tracks_in(Ind_2).StartEnd);
        f_12=intersect(f_1_ind(1):f_1_ind(2),f_2_ind(1):f_2_ind(2));
        Ratio=length(f_12)./[f_1_ind(2)-f_1_ind(1)+1 f_2_ind(2)-f_2_ind(1)+1];
        if f_1_ind(2)>f_2_ind(2)  || max(Ratio)>=0.9
            assignments(ci,:)=0;
        end
    end
end
assignments(assignments(:,1)==0,:)=[];

% % combine tracks
Low_tracks_out = TrackletsMergeOperation_V1(Low_tracks_in,assignments);

end

