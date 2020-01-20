function sim_datatools = processSimData(sim_path,to_run)
% Convert a simulation file into a struct containing all data
% Input: sim_path: absolute path for the simulation (.asim) file
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
    
    sour_folder = 'C:\Program Files (x86)\NeuroRobotic Technologies\AnimatLab\bin';
%     if ~isfile([pwd,'\Animatlab\SynergyWalking\SynergyWalking20200109_Standalone.asim'])
    if ~isfile(sim_path)
        error('Sim file doesn''t exist. Try running synergyProjectBuilder first.')
    else
        sim_path = [pwd,'\Animatlab\SynergyWalking\SynergyWalking20200109_Standalone.asim'];
%         sim_path = "G:\My Drive\Rat\SynergyControl\Animatlab\IndividualMuscleStim20190429_mod_Standalone.asim";
    end

    if to_run
        executable = ['"',sour_folder,'\AnimatSimulator" "',sim_path,'"'];
        [status, message] = system(executable);

        if status
            error(message)
            return
        end
    end

    fid = fopen(sim_path);
    sim_text = textscan(fid,'%s','delimiter','\n');
    fclose(fid);
    sim_text = [sim_text{:}];
    temp_filenames = sim_text(contains(sim_text,'<OutputFilename>'));
    filenames = extractBetween(temp_filenames,'<OutputFilename>','</OutputFilename>');
    sim_datatools = struct();

    for ii = 1:size(filenames,1)
        fpath = [char(fileparts(sim_path)),'\',filenames{ii}];
        if isfile(fpath)
            ds = importdata(fpath);
            sim_datatools(ii).name = filenames{ii}(1:end-4);
            if isfield(ds,'colheaders')
                sim_datatools(ii).time = ds.data(:,2);
                sim_datatools(ii).data = ds.data(:,3:end);
                sim_datatools(ii).data_cols = ds.colheaders(1,3:end);
            else
                sim_datatools(ii).time = ds.textdata(2:end,2);
                sim_datatools(ii).data = ds.data;
            end
        end
    end
end