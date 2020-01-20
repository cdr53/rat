function meshMatch(docpath)
    % For input docpath, checks that mesh files are pointing to local file locations
    % Input can be a path ofr a .asim Simulation file or .aproj  project file
    % Finds all instances of meshfile references and reinstantiates them to point to local files
    % Ensures that meshes load correctly for simulation

    fid = fopen(docpath);
    doc_text = textscan(fid,'%s','delimiter','\n');
    fclose(fid);
    doc_text = [doc_text{:}];
    
    meshInds = find(contains(doc_text,'<MeshFile>'));
    meshLines = extractBetween(doc_text(meshInds),'<MeshFile>','</MeshFile>');
    
    for ii = 1:length(meshLines)
        line = meshLines{ii};
        if ~contains(line,'\Animatlab\Meshes\')
            slashInds = strfind(line,'\');
            fileTrailer = line(slashInds(end-1):end);
            outline = ['<MeshFile>',pwd,'\Animatlab\Meshes\Rat',fileTrailer,'</MeshFile>'];
            doc_text{meshInds(ii)} = outline;
        end
    end
    
    convexInds = find(contains(doc_text,'<ConvexMeshFile>'));
    convexLines = extractBetween(doc_text(convexInds),'<ConvexMeshFile>','</ConvexMeshFile>');
    
    for ii = 1:length(convexLines)
        line = convexLines{ii};
        if ~contains(line,'\Animatlab\Meshes\')
            slashInds = strfind(line,'\');
            fileTrailer = line(slashInds(end-1):end);
            outline = ['<ConvexMeshFile>',pwd,'\Animatlab\Meshes\Rat',fileTrailer,'</ConvexMeshFile>'];
            doc_text{convexInds(ii)} = outline;
        end
    end
    
    fid = fopen(docpath,'w');
    formatSpec = '%s\n';
    nrows = size(doc_text,1);
    for row = 1:nrows
        fprintf(fid,formatSpec,doc_text{row,:});
    end
    fclose(fid);
end
