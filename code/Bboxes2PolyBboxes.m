function [PolyBboxes] = Bboxes2PolyBboxes(bboxes)
%   PolyBboxes | x y | reshape(~,4,[])

Point_1=bboxes(:,1:2);
Point_2=[bboxes(:,1) bboxes(:,2)+bboxes(:,4)];
Point_3=[bboxes(:,1)+bboxes(:,3) bboxes(:,2)+bboxes(:,4)];
Point_4=[bboxes(:,1)+bboxes(:,3) bboxes(:,2)];
Point_all=[Point_1 Point_2 Point_3 Point_4];
PolyBboxes=[reshape(Point_all(:,1:2:end)',[],1) reshape(Point_all(:,2:2:end)',[],1)];
end

