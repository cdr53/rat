function meshMatch(docpath)
    % Updates all mesh file paths for a given document and overwrites it.
    % Input: docpath: filepath to a Simulation file (.asim) or a Project file (.aproj)
    
    if ~isfile(docpath)
        error('meshMatch: File not found.')
    end
    
    %% Store the document text as a cell array
    fid = fopen(docpath);
    doc_text = textscan(fid,'%s','delimiter','\n');
    fclose(fid);
    doc_text = [doc_text{:}];
    
    %% Update all MeshFiles
    meshInds = find(contains(doc_text,'<MeshFile>'));
    meshLines = extractBetween(doc_text(meshInds),'<MeshFile>','</MeshFile>');
    
    for ii = 1:length(meshLines)
        line = meshLines{ii};
        slashInds = strfind(line,'\');
        fileTrailer = line(slashInds(end-1):end);
        outline = ['<MeshFile>',pwd,'\Animatlab\Meshes\Rat',fileTrailer,'</MeshFile>'];
        doc_text{meshInds(ii)} = outline;
    end
    
    %% Update all ConvexMeshFiles
    convexInds = find(contains(doc_text,'<ConvexMeshFile>'));
    convexLines = extractBetween(doc_text(convexInds),'<ConvexMeshFile>','</ConvexMeshFile>');
    
    for ii = 1:length(convexLines)
        line = convexLines{ii};
        slashInds = strfind(line,'\');
        fileTrailer = line(slashInds(end-1):end);
        outline = ['<ConvexMeshFile>',pwd,'\Animatlab\Meshes\Rat',fileTrailer,'</ConvexMeshFile>'];
        doc_text{convexInds(ii)} = outline;
    end
    
    %% Write updated document text to the original document
    fid = fopen(docpath,'w');
    formatSpec = '%s\n';
    nrows = size(doc_text,1);
    for row = 1:nrows
        fprintf(fid,formatSpec,doc_text{row,:});
    end
    fclose(fid);
end
