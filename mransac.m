clc; clear all;
input_filename = '03380.ply';
road_filename = 'road.ply';
curb_filename = 'curb.ply';

ply = pcread(input_filename);
points = ply.Location;


distances = sum(points.^2, 2);
[value, index] = min(distances);
roi = [-value value -value value -value value];
roi_indeces = findPointsInROI(ply,roi);

%find the primary plane (road)
[model,road_inlierIndices,road_outlierIndices] = pcfitplane(ply,0.02,'SampleIndices', roi_indeces, 'MaxNumTrials', 500);
plane_points = select(ply,road_inlierIndices);
%pcwrite(plane_points, road_filename);

%find the secondary plane (curb)
[model,curb_inlierIndices, curb_outlierIndices] = pcfitplane(ply,0.02,'SampleIndices', road_outlierIndices, 'MaxNumTrials', 500);
curb_points = select(ply,curb_inlierIndices);
%pcwrite(curb_points, curb_filename);

%find the rest
other_indeces = setdiff(road_outlierIndices, curb_inlierIndices);
other_points = select(ply,other_indeces);
%pcwrite(other_points, 'other.ply');


