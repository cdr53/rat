function sim_datatools = processSimData(sim_path,to_run)
% Convert a simulation file into a struct containing all data
% Input: sim_path: absolute path for a simulation (.asim) file
% Input: to_run: boolean to determine whether or not to execute the simulation file, regenerating output files
% Output: sim_datatools: struct containing all output data references in simulation file
    if nargin < 2
        if ~nargin
            error('Input simulation file path.')
        else
            to_run = 0;
        end
    end
    
    if ~ismember(to_run,[0,1])
        warning('to_run must be either 0 or 1. Setting to 0.')
        to_run = 0;
    end
    
    if ~isfile(sim_path)
        error('Sim file doesn''t exist. Try running synergyProjectBuilder first.')
    end

    if to_run
        sour_folder = 'C:\Program Files (x86)\NeuroRobotic Technologies\AnimatLab\bin';
        executable = ['"',sour_folder,'\AnimatSimulator" "',sim_path,'"'];
        [status, message] = system(executable);

        if status
            error(message)
            return
        end
    end

    sim_text = importdata(sim_path);
    temp_filenames = sim_text(contains(sim_text,'<OutputFilename>'));
    filenames = extractBetween(temp_filenames,'<OutputFilename>','</OutputFilename>');
    sim_datatools = struct();
    counter = 1;

    for ii = 1:size(filenames,1)
        fpath = [char(fileparts(sim_path)),'\',filenames{ii}];
        if isfile(fpath)
            
            ds = importdata(fpath);
            sim_datatools(counter).name = filenames{ii}(1:end-4);
            if isfield(ds,'colheaders')
                sim_datatools(counter).time = ds.data(:,2);
                sim_datatools(counter).data = ds.data(:,3:end);
                sim_datatools(counter).data_cols = ds.colheaders(1,3:end);
            else
                sim_datatools(counter).time = ds.textdata(2:end,2);
                sim_datatools(counter).data = ds.data;
            end
            counter = counter+1;
        else
            warning('%s does not exist. Try running the simulation file first. Set to_run to 1 and run again.\n',filenames{ii})
        end
    end
end