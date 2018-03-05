function ptCloud = createPointCloud(XYZ_ORG, RGB, limits)
% This function creates a pointcloud 'ptCloud' from an organized XYZ and a
% RGB image and filters out the points that are not situated between the
% limits.

%-- size of matrix
    [sz1,sz2,~] = size(XYZ_ORG);
%-- set the values of the elements that are not situated between the limits to inf
     for row = 1:sz1
         for column = 1:sz2
             if XYZ_ORG(row,column,3)<= limits(5) || XYZ_ORG(row,column,3)>= limits(6)
                 XYZ_ORG(row,column,3)=NaN;
             end
             if XYZ_ORG(row,column,2)<= limits(3) || XYZ_ORG(row,column,2)>= limits(4)
                 XYZ_ORG(row,column,3)=NaN;
             end
             if XYZ_ORG(row,column,1)<= limits(1) || XYZ_ORG(row,column,1)>= limits(2)
                 XYZ_ORG(row,column,3)=NaN;
             end
         end
     end
%-- create pointcloud
    ptCloud = pointCloud(XYZ_ORG,'Color',RGB);
%-- remove inf points
    [ptCloud,~]=removeInvalidPoints(ptCloud);
 end