function polyInter = OverlapPoly(polybox_1,polybox_2)
poly1=polyshape(polybox_1(:,1),polybox_1(:,2));
poly2=polyshape(polybox_2(:,1),polybox_2(:,2));
TR1 = overlaps(poly1,poly2);
if TR1==0
    polyInter=[];
else
    polyInter=intersect(poly1,poly2);
end
end