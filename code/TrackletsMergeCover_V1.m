function [Low_tracks_out] = TrackletsMergeCover_V1(Low_tracks_in,ObjN,ThreSeg,ThreCover)
%  find and delete the tracklets which are totally covered by others
%  satisfied the criteria: 1) more than ObjN tracklets covered this
%  tracklet; 2) covering threshould (ThreSeg) for startpoint and endpoint;
%  3)covering threshould ratio 
%  Low_tracks_in  |  input tracelets
%  ObjN  |  number of objects

%  -----------------------------------------
TR1=[Low_tracks_in(:).StartEnd]';
TR1=double(TR1);
% Low_tracks_dura=TR1(:,2)-TR1(:,1);

CoverM=ones(length(Low_tracks_in));
for ci=1:length(Low_tracks_in)-1  % 
    for cj=ci+1:length(Low_tracks_in)-1 % covering detection
        if TR1(ci,1)+ThreSeg(1)<=TR1(cj,1) && TR1(ci,2)>=TR1(cj,2)+ThreSeg(1)
            CoverM(ci,cj)=(TR1(cj,2)-TR1(cj,1))/(TR1(ci,2)-TR1(ci,1));
        end
    end
end
CoverM_sort=sort(CoverM,'ascend');
CoverM_sort_obj=CoverM_sort(ObjN,:);

Low_tracks_out=Low_tracks_in;
Low_tracks_out( CoverM_sort_obj<=ThreCover )=[];

end

