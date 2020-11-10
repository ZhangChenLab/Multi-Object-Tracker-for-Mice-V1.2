function [Low_tracks_post] = TrackletsMergeOperation_V1(Low_tracks_pre,assignments)
%   Low_tracks_pre  | tracklets waiting for fusion
%   assignments     | index of tracklets fusion
%   Low_tracks_post | tracklets after fusion

Low_tracks_post = Low_tracks_pre;

if ~isempty(assignments)
    [~,sb]=sort(assignments(:,2),'descend');  % from small to bigger
    assignments=assignments(sb,:);
    for ci=1:size(assignments,1)
        Ind_1=assignments(ci,1);
        Ind_2=assignments(ci,2);
        frame_1_ind=Low_tracks_pre(Ind_1).frame;
        frame_2_ind=Low_tracks_pre(Ind_2).frame;
        if frame_1_ind(end)>=frame_2_ind(1)  % 
            TR1=find(frame_1_ind==frame_2_ind(1));
            Low_tracks_pre(Ind_1).frame=[Low_tracks_pre(Ind_1).frame(1:TR1-1); Low_tracks_pre(Ind_2).frame];
            Low_tracks_pre(Ind_1).polybbox=cat(3,Low_tracks_pre(Ind_1).polybbox(:,:,1:TR1-1), Low_tracks_pre(Ind_2).polybbox);
%             Low_tracks_pre(Ind_1).Ind=[Low_tracks_pre(Ind_1).Ind(1:TR1-1,:); ...
%                          Low_tracks_pre(Ind_1).Ind(end)+Low_tracks_pre(Ind_2).Ind];
%             Low_tracks_pre(Ind_1).Fun{end+1,2}=Low_tracks_pre(Ind_1).frame(TR1-1);  % 
        elseif frame_1_ind(end)+1==frame_2_ind(1)
            Low_tracks_pre(Ind_1).frame=[Low_tracks_pre(Ind_1).frame; Low_tracks_pre(Ind_2).frame];
            Low_tracks_pre(Ind_1).polybbox=cat(3,Low_tracks_pre(Ind_1).polybbox, Low_tracks_pre(Ind_2).polybbox);
%             Low_tracks_pre(Ind_1).Ind=[Low_tracks_pre(Ind_1).Ind; ...
%                          Low_tracks_pre(Ind_1).Ind(end)+Low_tracks_pre(Ind_2).Ind];
%             Low_tracks_pre(Ind_1).Fun{end+1,2}=Low_tracks_pre(Ind_1).frame(end);   % 
        else
            TR_N=frame_2_ind(1)-frame_1_ind(end);
            polybbox_1=double(Low_tracks_pre(Ind_1).polybbox(:,:,end));
            polybbox_2=double(Low_tracks_pre(Ind_2).polybbox(:,:,1));
            polybbox_LS=zeros(4,2,TR_N+1);
            for pi=1:4
                for pj=1:2
                    polybbox_LS(pi,pj,:)=linspace(polybbox_1(pi,pj),polybbox_2(pi,pj),TR_N+1);
                end
            end
            polybbox_LS(:,:,[1 end])=[];
            Low_tracks_pre(Ind_1).frame=[Low_tracks_pre(Ind_1).frame;frame_1_ind(end)+(1:TR_N-1)'; Low_tracks_pre(Ind_2).frame];
            Low_tracks_pre(Ind_1).polybbox=cat(3,Low_tracks_pre(Ind_1).polybbox,uint32(polybbox_LS), Low_tracks_pre(Ind_2).polybbox);
%             Low_tracks_pre(Ind_1).Ind=[Low_tracks_pre(Ind_1).Ind; ...
%                          (Low_tracks_pre(Ind_1).Ind(end)+1)*ones(size(polybbox_LS,1),1,'uint32'); ...
%                          Low_tracks_pre(Ind_1).Ind(end)+Low_tracks_pre(Ind_2).Ind];
%             Low_tracks_pre(Ind_1).Fun{end+1,2}=Low_tracks_pre(Ind_1).frame(end);   % 
        end
        Low_tracks_pre(Ind_1).StartEnd=[frame_1_ind(1); frame_2_ind(end)];
%         Low_tracks_pre(Ind_1).Num=Low_tracks_pre(Ind_1).Num+Low_tracks_pre(Ind_2).Num;
%         Low_tracks_pre(Ind_1).Fun{end,1}=CharFun;
%         if length(Low_tracks_pre(Ind_1).Ind)~=length(Low_tracks_pre(Ind_1).frame)
%             disp('err')
%         end
%         if ~isempty(Low_tracks_pre(Ind_2).Fun)
%             Low_tracks_pre(Ind_1).Fun(end+1:end+size(Low_tracks_pre(Ind_2).Fun,1),:)=Low_tracks_pre(Ind_2).Fun;
%         end
    end
    
    Low_tracks_post=Low_tracks_pre;
    Low_tracks_post( assignments(:,2) )=[];
end

end

