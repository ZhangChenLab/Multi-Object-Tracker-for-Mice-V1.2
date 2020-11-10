function [Low_tracks_out] = TrackletsMergeMultiCamera_V2(TrackletsCamAll,ParaStructMultiCam)
%  duration criteria: 1)startpoint and endpoint threshould;2) duration
%  threshold
%  IoU criteria: 1) the difference between the most and secondly related
%  IoU ThreM(1); 2) the most related IoU > ThreM(2)
%  Low_tracks   tracklets
%  ThreSeg | gap overlap length 
%  ThreMDist | Distance threshold | 2nd/1st(ascend) 1st threshold

assignments=[];
% LenOverlapThre=[10 5 10];  % threshold for duration, overlap, and gap
% ThreMDist=[1.5 40 40];     % distance threshold
LenOverlapThre=ParaStructMultiCam.LenOverlapThre;
ThreMDist=ParaStructMultiCam.ThreMDist;

tracelets_0=TrackletsCamAll{1};
TRDura_0=[tracelets_0(:).StartEnd]';
for camera_i=2:length(TrackletsCamAll)
tracelets_1=TrackletsCamAll{camera_i};

MergingDist=inf(length(tracelets_0),length(tracelets_1));
for ci=1:length(tracelets_0)
    TR_StartEnd_0=tracelets_0(ci).StartEnd;
    LenFlag_0=TR_StartEnd_0(2)-TR_StartEnd_0(1)>=LenOverlapThre(1);
    for cj=1:length(tracelets_1)
        TR_StartEnd_1=tracelets_1(cj).StartEnd;
        LenFlag_1=TR_StartEnd_1(2)-TR_StartEnd_1(1)>=LenOverlapThre(1);
        if   LenFlag_0 && LenFlag_1
            Flag_1=(TR_StartEnd_0(2)<TR_StartEnd_1(2)) && (TR_StartEnd_0(2)>TR_StartEnd_1(1));
            Flag_2=(TR_StartEnd_0(1)<TR_StartEnd_1(2)) && (TR_StartEnd_0(1)>TR_StartEnd_1(1));
            Flag_3=max(Flag_1,Flag_2);
            if  Flag_3  % two tracklets are overlapd && not contained
                Overlap_start=max([TR_StartEnd_0(1) TR_StartEnd_1(1)]);
                Overlap_end=min([TR_StartEnd_0(2) TR_StartEnd_1(2)]);
                Overlap_N=Overlap_end-Overlap_start;
                if  Overlap_N>=LenOverlapThre(2)
                    Ind_0=Overlap_start-TR_StartEnd_0(1)+1;  % ovlap index
                    Ind_1=Overlap_start-TR_StartEnd_1(1)+1;
                    
                    Cen_0=mean(tracelets_0(ci).polybbox(:,:,Ind_0:Ind_0+Overlap_N),1);
                    Cen_0=permute(Cen_0,[3 2 1]);
                    Cen_1=mean(tracelets_1(cj).polybbox(:,:,Ind_1:Ind_1+Overlap_N),1);
                    Cen_1=permute(Cen_1,[3 2 1]);
                    DeltaXY=Cen_0-Cen_1;
                    DeltaXY=sqrt(sum(DeltaXY.*DeltaXY,2));
                    MergingDist(ci,cj)=mean(DeltaXY);
                end
            end
        end
    end
end
MergingDist=MergingDist';
[ca,cb]=sort(MergingDist,1,'ascend');
Flag_0=ca(2,:)./ca(1,:);
Flag_0(isnan(Flag_0))=0;
Flag_1=Flag_0>=ThreMDist(1);
Flag_0(Flag_0==inf)=2;
Flag_2=ca(1,:)<=ThreMDist(2)*(1+Flag_0);
Flag_3=ca(2,:)>=ThreMDist(3);
FlagAll=Flag_1.*Flag_2.*Flag_3;

SparsMX=find(FlagAll==1);
SparseMY=cb(1,SparsMX);
SparseM=sparse(SparseMY,SparsMX,ones(1,length(SparsMX)),...
    length(tracelets_1),length(tracelets_0));
MergingDist_01=full(SparseM);

TR1=sum(MergingDist_01,1);
MergingDist(:,TR1>=2)=inf;  % camera-1 --> camera X | many-->1 | delete 1-->many
MergingDist_01(:,TR1>=2)=0;
for cj=1:length(tracelets_1)
    TR1=find(MergingDist_01(cj,:)==1);
    if length(TR1)>=2
        TR2=[TR1(1:end-1); TR1(2:end)]';
        TRDura_1=double([TRDura_0(TR2(:,1),2) TRDura_0(TR2(:,2),1)]);
        TRDura_2=TRDura_1(:,2)-TRDura_1(:,1);
        TRDura_Flag_1=(TRDura_2<=0).*( TRDura_2>=-10);  % overlap 10
        TRDura_Flag_2=(TRDura_2>=0).*( TRDura_2<=20);  % gap 10
        TRDura_Flag=max(TRDura_Flag_1,TRDura_Flag_2);
        TRDura_Flag=TRDura_Flag==1;
        TR2=TR2(TRDura_Flag,:);
        
        TR3=MergingDist(cj,TR1);
        TR3_1=[TR3(1:end-1); TR3(2:end)]';
        TR3_1=TR3_1(TRDura_Flag,:);
        assignments=[assignments; TR2 camera_i*ones(size(TR2,1),1) TR3_1]; % 
    end
end
end

% if ~isempty(assignments)
%     [~,ia,~]=unique(assignments(:,1:2),'row');
%     assignments= assignments(ia,:);
%     if length(unique(assignments(:,1)))~=size(assignments,1) || ...
%             length(unique(assignments(:,2)))~=size(assignments,1)
%         disp('----ERROR in Merging----')
%     end
% end
if ~isempty(assignments)
    [~,ia,~]=unique(assignments(:,1:2),'row');
    assignments= assignments(ia,:);
    if length(unique(assignments(:,1)))~=size(assignments,1) || ...
            length(unique(assignments(:,2)))~=size(assignments,1)
        unique_1=assignments(1,1);
        unique_2=assignments(1,2);
        unique_ind=1;
        for ci=2:size(assignments,1)
            if ~sum(unique_1==assignments(ci,1)) && ~sum(unique_2==assignments(ci,2))
                unique_1=[unique_1; assignments(ci,1)];
                unique_2=[unique_2; assignments(ci,2)];
                unique_ind=[unique_ind; ci];
            end
        end
        assignments=assignments(unique_ind,:);
    end
    
end
Flag=zeros(size(assignments,1),1);
for ci=1:size(assignments,1)
    TRDura=[tracelets_0(assignments(ci,1:2)).StartEnd];
    TRDura=double(TRDura);
    if TRDura(2,1)<TRDura(2,2) && abs(TRDura(2,1)-TRDura(1,2))<=LenOverlapThre(3)  % LenThre
        Flag(ci,1)=1;
    end
end
assignments=assignments(Flag==1,:);
disp([num2str(size(assignments,1)) ' combinations by multi-camera done.'])


%% assignments merge  performance
% prename=prename_T{1};
% load([prename '.mat'],'VideoStruct')
% for ci=1:size(assignments,1)
% TRAssign=assignments(ci,1:2);
% tracelets_pair=tracelets_0(TRAssign);
% TR_StartEnd=[tracelets_pair.StartEnd];
% TR_StartEnd=reshape(TR_StartEnd,[],1);
% FrameInd0=[min(TR_StartEnd(2:3)) max(TR_StartEnd(2:3))];
% NFrame=10;
% FrameInd=[FrameInd0(1)-NFrame FrameInd0(2)+NFrame];
% TrackletsPlot_V1(tracelets_pair,VideoStruct,FrameInd(1):FrameInd(2),'Video');
% % TrackletsPlot_V1(tracelets_pair,[],FrameInd(1):FrameInd(2),'Lines')
% % TrackletsPlot_V1(tracelets_pair,[],FrameInd(1):FrameInd(2),'Trace2DTrace')
% end

%% do assignment and merge 
Low_tracks_out=tracelets_0;
if ~isempty(assignments)
    TR_StartEnd=[tracelets_0(reshape(assignments(:,1:2)',[],1)).StartEnd];
    TR_StartEnd=reshape(TR_StartEnd,4,[]);
    TR_StartEnd=double(TR_StartEnd');
    assignments=[assignments TR_StartEnd(:,2:3)];
    % % combine tracks
    Low_tracks_out = TrackletsMergeOperation_V1(tracelets_0,assignments(:,1:2));
end

end

