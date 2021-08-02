clc;clear all;


kkk = 1; %saliency mapping

%t = readtable('/home/stagakis/Desktop/multi_agent_realistic_potholes/ego0/steering_true.txt', 'ReadVariableNames', false);
%A = table2array(t);

%propath = '/home/stagakis/Desktop/multi_agent_realistic_potholes/ego0/sensor.lidar.ray_cast/';
%pcl_files = dir(strcat(propath,'*.ply'));

propath = '/home/stagakis/Desktop/road_with_pothole_generation/completed_potholes/';
pcl_files = dir(strcat(propath,'colored_*.ply'));

for k = 1:size(pcl_files)
%for k = 100:200

    clearvars -except k pcl_files t A kkk propath
    %%% we read the geometry of the scene
    file = pcl_files(k).name    
    frame = file(1:size(file,2)-4);
    extension = file(size(file,2)-3:end);
    
    fullpath = strcat(propath,file);
    full_file_name = strcat(propath,frame);
    
    if extension == ".ply"
        oo = plyread(fullpath);
        ptCloud = pointCloud([oo.vertex.x, oo.vertex.y, oo.vertex.z]);
    else %.mat
        oo = load(fullpath);
        ptCloud = pointCloud(oo.xyzPoints);

    end

    
    %gridStep = 5;
    %ptCloud = pcdownsample(ptCloud,'gridAverage',gridStep);
    
    vertices(:,1) = ptCloud.Location(:,1);
    vertices(:,2) = ptCloud.Location(:,2);
    vertices(:,3) = ptCloud.Location(:,3);
    
    tic
    %%%%%%%%%%%%% We estimate the neighbors %%%%%%%%%%%%%%%%%%%%%%%
    apoints = [ vertices(:,1), vertices(:,2), vertices(:,3)]; 

    numofsalientneig = 30;
    apoints = double(apoints);
    akdtreeobj = KDTreeSearcher(apoints,'distance','euclidean');
    aAdjOfPoints = knnsearch(akdtreeobj,apoints,'k',(numofsalientneig+1));
    AdjOfPointsa3 = knnsearch(akdtreeobj,apoints,'k',(numofsalientneig+1));
    aAdjOfPoints = aAdjOfPoints(:,2:end);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%% We estimate the normals of each point
    [normalvectors,curvature] = findPointNormals(vertices,[],[0,0,0],true);
    featuresize = size(vertices,1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    heatmap = zeros(size(vertices,1),1);

    %%%%%%%%%%%%%% We estimate spectral saliency %%%%%%%%%%%%%%%%%%%
    for i = 1:featuresize

         for j = 1:numofsalientneig 
            nn1(j,:) = normalvectors(AdjOfPointsa3(i,j),:);
         end

         convn1 = nn1'*nn1;

         [vb lb] = eig(convn1);

         calll(i,1) = lb(1,1);
         calll(i,2) = lb(2,2);
         calll(i,3) = lb(3,3);

         heatmap(i,1) = 1./norm(calll(i,:));
         %i
    end

    norm_heatmap = (heatmap - min(heatmap)) / ( max(heatmap) - min(heatmap) );

    color11 = zeros(size(vertices,1),3);
    a11 = zeros(size(vertices,1),1);
    
    range11 = max(norm_heatmap); 
    cmp = colormap(jet);
    sizecmp2 = size(cmp,1);
    step11 = range11/64; %sizecmp2;
    toc
    for i = 1:featuresize
        k = 1;
        while((norm_heatmap(i,1) > k*step11) && k < 64) %% we use only 16 from a 64 colormap  
            k = k + 1;
        end
        color11(i,:) = cmp((k-1) + 1,:);
        a11(i,1) =  k; %labeling
     end

    saliency_colors = colorizepointcloud(vertices, a11);
    ending11 = '_eigen_colormap.obj'; 
    path1 = strcat(full_file_name,ending11);
    fileID = fopen(path1, 'w');
    for i = 1:size(vertices,1)
         fprintf(fileID,'v %f %f %f %f %f %f\n',vertices(i,1), vertices(i,2), vertices(i,3), saliency_colors(i,1), saliency_colors(i,2), saliency_colors(i,3));
    end
    fclose(fileID);

    ending111 = '_eigen_binary.obj'; 
    path1 = strcat(full_file_name,ending111);
    fileID = fopen(path1, 'w');
    for i = 1:size(vertices,1)
        if a11(i,1) <= 10
         fprintf(fileID,'v %f %f %f %f %f %f\n',vertices(i,1), vertices(i,2), vertices(i,3), 0, 0, 1);
        else
         fprintf(fileID,'v %f %f %f %f %f %f\n',vertices(i,1), vertices(i,2), vertices(i,3), 1, 0, 0);
        end
    end
    fclose(fileID);
end
