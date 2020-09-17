function simStruct = processSimData(sim_path)
% Convert a simulation file into a struct containing all data
% Input: sim_path: absolute path for a simulation (.asim) file
% Output: simStruct: struct containing all output data references in simulation file
    if nargin ~= 1
        error('run_simfile: Enter the path to an .asim file.')
    end

    if isstring(sim_path)
        sim_path = char(sim_path);
        if ~contains(sim_path,'.asim')
            error('procesSimData: Provided file path is not an .asim file.')
        end
    end
   
    if ~isfile(sim_path)
        error('processSimData: Sim file doesn''t exist.')
    end

    if isstring(sim_path)
        sim_path = char(sim_path);
    end
    
    % Pre-process .asim file to ensure that mesh filepaths and mass values are correct
%     meshMatch(sim_path);
%     massCheck(sim_path);
    
    % Run the simulation file
    sour_folder = 'C:\AnimatLabSDK\AnimatLabPublicSource\bin';
    %sour_folder = 'C:\Program Files (x86)\NeuroRobotic Technologies\AnimatLab\bin';
    executable = ['"',sour_folder,'\AnimatSimulator" "',sim_path,'"'];
    [status, message] = system(executable);
    if status
        error(message)
        return
    end

    % Process output simulation rsults, as stored in .txt files related to the .aform datatools
    sim_text = importdata(sim_path);
    temp_filenames = sim_text(contains(sim_text,'<OutputFilename>'));
    filenames = extractBetween(temp_filenames,'<OutputFilename>','</OutputFilename>');
    simStruct = struct();
    counter = 1;

    for ii = 1:size(filenames,1)
        fpath = [char(fileparts(sim_path)),'\',filenames{ii}];
        if isfile(fpath)
            ds = importdata(fpath);
            if ~isempty(ds)
                simStruct(counter).name = filenames{ii}(1:end-4);
                if isfield(ds,'colheaders')
                    simStruct(counter).time = ds.data(:,2);
                    simStruct(counter).data = ds.data(:,3:end);
                    simStruct(counter).data_cols = ds.colheaders(1,3:end);
                else
                    simStruct(counter).time = ds.textdata(2:end,2);
                    simStruct(counter).data = ds.data;
                end
                counter = counter+1;
            end
        else
            warning('%s does not exist. Try running the simulation file first. Set to_run to 1 and run again.\n',filenames{ii})
        end
    end
end