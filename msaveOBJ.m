function msaveOBJ(filaname, vertices, colors)
    fileID = fopen(filaname, 'w');
    for i = 1:size(vertices,1)
         fprintf(fileID,'v %f %f %f %f %f %f\n',vertices(i,1), vertices(i,2), vertices(i,3), colors(i,1), colors(i,2), colors(i,3));
    end
    fclose(fileID);
end
