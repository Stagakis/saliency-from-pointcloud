clc;clear all;
setup_fastRPCA()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% we read the steering angle from the file
t = readtable('E:/Windows/ego0/steering_true.txt', 'ReadVariableNames', false);
A = table2array(t);

kkk = 1;  %saliency mapping
kkk1 = 1; %segmentation with rotation based on the steering
kkk2 = 1; %segmentation without rotation based on the steering

for i = 1:size(A,2)-1    
    AA(1,i) = -deg2rad(str2double(A{i})); 
end

first_frame = 03045; %02956;

for frame = 3045:3045 %02956:3380
    %%% we read the geometry of the scene
    myfilename = int2str(frame);
    propath = 'E:/Windows/ego0/sensor.lidar.ray_cast/0';
    ending = '.ply';
    fullpath = strcat(propath,myfilename,ending);

    oo = plyread(fullpath);

    vertices(:,1) = oo.vertex.x;
    vertices(:,2) = oo.vertex.y;
    vertices(:,3) = oo.vertex.z;
    
        
    %%%%%%%%% READ FROM .MAT FILE INSTEAD
    mat_pcl = load('C:\Users\Stagakis\Desktop\stereo_pothole_datasets\dataset2\ptcloud\01.mat');
    dims = size(mat_pcl.xyzPoints);
    vertices = reshape(mat_pcl.xyzPoints, [dims(1)*dims(2),dims(3)]);
    infinites = find(isinf(vertices(:,1)));
    vertices(infinites, :) = []; %delete infinites
    vertices_pcl = pointCloud(vertices);
    vertices_pcl = pcdownsample(vertices_pcl,'random',0.005);
    vertices = vertices_pcl.Location;
    %%%%%%%%
    
    tic
    %%%%%%%%%%%%% We estimate the neighbors %%%%%%%%%%%%%%%%%%%%%%%
    apoints = [ vertices(:,1), vertices(:,2), vertices(:,3)]; 

    numofsalientneig = 150;
    apoints = double(apoints);
    akdtreeobj = KDTreeSearcher(apoints,'distance','euclidean');
    %aAdjOfPoints = knnsearch(akdtreeobj,apoints,'k',(numofsalientneig+1));
    AdjOfPointsa3 = knnsearch(akdtreeobj,apoints,'k',(numofsalientneig+1)); 
    %aAdjOfPoints = aAdjOfPoints(:,2:end);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%% We estimate the normals of each point
    [normalvectors,curvature] = findPointNormals(vertices,[],[0,0,0],true);
    featuresize = size(vertices,1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


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
    end

    norm_heatmap = (heatmap - min(heatmap)) / ( max(heatmap) - min(heatmap) );
    toc
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% We estimate the features based on RPCA
    tic
    X = zeros(featuresize,3*(numofsalientneig));
    for i = 1:featuresize
        for j = 1:numofsalientneig
            X(i,3*(j-1)+1) = normalvectors(AdjOfPointsa3(i,j),1);
            X(i,3*(j-1)+2) = normalvectors(AdjOfPointsa3(i,j),2);
            X(i,3*(j-1)+3) = normalvectors(AdjOfPointsa3(i,j),3);
        end
%     i
    end

    lambda = 1/sqrt(max(size(X)));

    opts = struct('sum',false,'max',true);
    
       [L,S] = solver_RPCA_constrained(X, lambda, 1e-5, [], opts);
       %[L,S] = inexact_alm_rpca(X, lambda, 1e-5);
    toc

    for i = 1:featuresize
        valofS(i,1) = sqrt((S(i,1))^2+(S(i,2))^2+(S(i,3))^2);
    end

    dd = valofS;

    ddd = (dd - min(dd)) / ( max(dd) - min(dd) ); %  dd;%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%%%%%%%%%%%%% combination of two approaches %%%%%%%%
    ddd2 = (ddd + norm_heatmap)./2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%% We create the saliency map and the labeling %%%%%%%%%%%%%%%
    range11 = max(norm_heatmap); 
    cmp = colormap(jet);
    sizecmp2 = size(cmp,1);
    step11 = range11/64; %sizecmp2;

     for i = 1:featuresize
         k = 1;
         while((norm_heatmap(i,1) > k*step11) && k < 64) %% we use only 16 from a 64 colormap  
             k = k + 1;
         end
         color11(i,:) = cmp((k-1) + 1,:);
         a11(i,1) =  k; %labeling
     end


    range21 = max(ddd); 
    cmp = colormap(jet);
    sizecmp2 = size(cmp,1);
    step21 = range21/64; %sizecmp2;

     for i = 1:featuresize
         k = 1;
         while((ddd(i,1) > k*step21) && k < 64) %% we use only 16 from a 64 colormap  
             k = k + 1;
         end
         color21(i,:) = cmp((k-1) + 1,:);
         a21(i,1) =  k; %labeling
     end
     

    range2 = max(ddd2); 
    cmp = colormap(jet);
    sizecmp2 = size(cmp,1);
    step2 = range2/64; %sizecmp2;

     for i = 1:featuresize
         k = 1;
         while((ddd2(i,1) > k*step2) && k < 64) %% we use only 16 from a 64 colormap  
             k = k + 1;
         end
         color(i,:) = cmp((k-1) + 1,:);
         a(i,1) =  k; %labeling
     end

    %%%%%%% Default sintetagmenes 
    %%% vertices(i,1) < 3 && vertices(i,1) > -5.3 %% ta oria toy dromoy
    %%% vertices(i,2) < 0 mprosta
    %%% vertices(i,3) 
   
    vertices_rotated(:,1) = vertices(:,1)*cos(AA(1,frame-first_frame+1)) - vertices(:,2)*sin(AA(1,frame-first_frame+1));
    vertices_rotated(:,2) = vertices(:,1)*sin(AA(1,frame-first_frame+1)) + vertices(:,2)*cos(AA(1,frame-first_frame+1));
    vertices_rotated(:,3) = vertices(:,3); 

    newvertices = vertices_rotated*[0 -1 0;1 0 0; 0 0 -1]; 
    newvertices_without = vertices*[0 -1 0;1 0 0; 0 0 -1]; 

    if kkk == 1
        msaveOBJ(strcat(myfilename,'_eigen_colormap.obj'), newvertices, color11);

        eigen_binary_colors = repmat([1 0 0],size(newvertices,1),1); %red by default
        eigen_binary_colors((a11 == 1), :) = repmat([0 0 1], sum((a11 == 1)), 1);
        msaveOBJ(strcat(myfilename,'_eigen_binary.obj'), newvertices, eigen_binary_colors);

        msaveOBJ(strcat(myfilename,'_rpca_colormap.obj'), newvertices, color21);

        rpca_binary_colors = repmat([1 0 0],size(newvertices,1),1); %red by default
        rpca_binary_colors((a21 == 1), :) = repmat([0 0 1], sum((a21 == 1)), 1);
        msaveOBJ(strcat(myfilename,'_rpca_binary.obj'), newvertices, rpca_binary_colors);

        msaveOBJ(strcat(myfilename,'_saliency_colormap.obj'), newvertices, color);
    end

    if kkk1 == 1
        saliency_colors = colorizepointcloud(newvertices, a);
        msaveOBJ(strcat(myfilename,'_saliency_segmentation.obj'), newvertices, saliency_colors);   
    end
    
    if kkk2 == 1
        saliency_colors = colorizepointcloud(newvertices_without, a);
        msaveOBJ(strcat(myfilename,'_saliency_segmentation_without_mine.obj'), newvertices_without, saliency_colors);   
    end

    occupancy_factor = 0;
    
    for i = 1:size(newvertices,1)
        if a(i,1) == 1 && newvertices(i,1) < 1 && newvertices(i,1) > -6.3  && newvertices(i,3) > 2.3 && newvertices(i,3) < 2.4 && newvertices(i,2) < -3 %% mple dromos
            occupancy_factor = occupancy_factor + 1/(norm(newvertices(i,:),2)); %%% twra einai me aplo antistrofo tis apostasis, 8a mporouse na exei kai vari analoga to obstacle tis skinis kai tin gwnia toy simiou (oxi mono eyklidia apostasi)
        end
    end
    
     occ(frame-first_frame+1, 1) = occupancy_factor;
    
    clearvars -except occ frame A AA first_frame kkk kkk1 kkk2 vertices
end
    
% save('occupancy_end.mat','occ')






