function [CostIoU,CostDist] = TrackletsMergeCostM_V1(Low_tracks_in,ThreSeg)
%UNTITLED8 此处显示有关此函数的摘要
%   此处显示详细说明  PolyBbox

CostIoU=inf(length(Low_tracks_in));
CostDist=inf(length(Low_tracks_in));
DeltaMag=[];  % average delta
for ci=1:length(Low_tracks_in)
    TR1=cell(1,2);
    TR1{1,1}=Low_tracks_in(ci).polybbox(:,:,1:4);
    TR1{1,2}=Low_tracks_in(ci).polybbox(:,:,end-3:end);
    for cj=1:2
        TR2=permute(mean(TR1{1,cj},1),[3 2 1]);
        TR3=TR2(2:end,:)-TR2(1:end-1,:);
        TR4=mean(TR3);
        DeltaMag=[DeltaMag; sqrt(sum(TR4.*TR4))];
    end
end
DeltaMagThre=prctile(DeltaMag,70);   

similaModi_T=[];
for ci=1:length(Low_tracks_in)-1
    for cj=ci+1:length(Low_tracks_in)
%         flag=false;
        loc_end=Low_tracks_in(ci).StartEnd(2); 
        loc_start=Low_tracks_in(cj).StartEnd(1);  
        TR1=cell(1,2);   % weighting of velocity and angle
        TR1{1,1}=Low_tracks_in(ci).polybbox(:,:,1:4);
        TR1{1,2}=Low_tracks_in(cj).polybbox(:,:,end-3:end);
        DeltaXYMag=[];
        for si=1:2
            TR2=permute(mean(TR1{1,si},1),[3 2 1]);
            TR3=TR2(2:end,:)-TR2(1:end-1,:);
            TR4=mean(TR3);
            DeltaXYMag=[DeltaXYMag; TR4 sqrt(sum(TR4.*TR4))];
        end
        theta=sum(DeltaXYMag(1,1:2).*DeltaXYMag(2,1:2))/DeltaXYMag(1,3)/DeltaXYMag(2,3);
        simila=1-acos(theta)/pi;
        if ~isreal(simila)
            simila=real(simila);
        end
        if isnan(simila)  % stand still would lead to theta, simila=nan
            simila=0;
        end
        TR_mag=min([mean(DeltaXYMag(:,3))/DeltaMagThre 1]);
        similaModi=(0.8+simila*0.1)*(0.8+TR_mag*0.1);
        similaModi_T=[similaModi_T; similaModi];
        
        if loc_end < loc_start  % gap | end vs start
            flag=abs(double(loc_end)-double(loc_start))<=ThreSeg(1); % gap
            if flag
                Reframe=Low_tracks_in(ci).polybbox(:,:,end);
                GTframe=Low_tracks_in(cj).polybbox(:,:,1);
                TR1=mean(Reframe)-mean(GTframe);
                CostDist(ci,cj)=sqrt(sum(TR1.*TR1));
                
                cost=1;
                polyInter = OverlapPoly(Reframe,GTframe);
                if ~isempty(polyInter)
                    ReframeArea=area(polyshape(Reframe));
                    GTframeArea=area(polyshape(GTframe));
                    polyInterArea=area(polyInter);
                    cost=1-polyInterArea/(ReframeArea+...
                        GTframeArea-polyInterArea);
                end
%                 CostIoU(ci,cj)=cost;
                CostIoU(ci,cj)=1-(1-cost)*similaModi;
            end
        else                 % overlap | mean of all overlap point 
            loc_startend=Low_tracks_in(cj).StartEnd(2);
            Overlap_N=min([loc_end  loc_startend])- loc_start;
            CostDist_N=zeros(1,Overlap_N+1);
            CostIoU_N=ones(1,Overlap_N+1);
            flag=abs(double(loc_end)-double(loc_start))<=ThreSeg(2);  % overlap
            if flag
                for ov_i=0:Overlap_N
                    Reframe=Low_tracks_in(ci).polybbox(:,:,end+ov_i-Overlap_N);
                    GTframe=Low_tracks_in(cj).polybbox(:,:,ov_i+1);
                    TR1=mean(Reframe)-mean(GTframe);
                    CostDist_N(1,ov_i+1)=sqrt(sum(TR1.*TR1));
                    cost=1;
                    polyInter = OverlapPoly(Reframe,GTframe);
                    if ~isempty(polyInter)
                        ReframeArea=area(polyshape(Reframe));
                        GTframeArea=area(polyshape(GTframe));
                        polyInterArea=area(polyInter);
                        cost=1-polyInterArea/(ReframeArea+...
                            GTframeArea-polyInterArea);
                    end
                    CostIoU_N(1,ov_i+1)=cost;
                end
                CostDist(ci,cj)=mean(CostDist_N);
%                 CostIoU(ci,cj)=mean(CostIoU_N);
                CostIoU(ci,cj)=1-(1-mean(CostIoU_N))*similaModi;
            end
        end
    end
end

end

