function project_file = inject_joint_waveforms(project_file,equations,end_time)
    % Injects joint waveform equations into joint motor stimuli and overwrites the file. 
    % Input: filepath: simulation .asim file path
    % Input: equations: cell array of sum of sines equations for hip, knee, and ankle
    % Input: end_time: end time of simulation
    
%     project_file = importdata(filepath);
    hip_ind = find(contains(project_file,'<Name>Hipwalking'))+10;
    knee_ind = find(contains(project_file,'<Name>Kneewalking'))+10;
    ankle_ind = find(contains(project_file,'<Name>xAnklewalking'))+10;
    equation_inds = [hip_ind,knee_ind,ankle_ind];
    simtime_ind = find(contains(project_file,'<APIFile/>'))+1;
    plot_ind = find(contains(project_file,'<OutputFilename>JointMotion.txt</OutputFilename>'))+6;
    
    % Optional: disable joint limits
    limit_enable_ind = contains(project_file,'<EnableLimits>True</EnableLimits>');
    project_file(limit_enable_ind) = deal({'<EnableLimits>False</EnableLimits>'});
    
    project_file{simtime_ind} = number_injector(project_file{simtime_ind},num2str(end_time));
    %project_file{plot_ind} = number_injector(project_file{plot_ind},num2str(end_time));
    
    for i = 1:length(equation_inds)
        old_eq_line = project_file{equation_inds(i)};
        old_endtime_line = project_file{equation_inds(i)-2};
        project_file{equation_inds(i)} = number_injector(old_eq_line,equations{i});
        project_file{equation_inds(i)-2} = number_injector(old_endtime_line,num2str(end_time));
    end

%     fileID = fopen(filepath,'w');
%     fprintf(fileID,'%s\n',project_file{:});
% %     formatSpec = '%s\n';
% %     nrows = size(project_file);
% %     for row = 1:nrows
% %         fprintf(fileID,formatSpec,project_file{row,:});
% %     end
%     fclose(fileID);
    
    function new_line = number_injector(old_line,new_eq)
        geq = find(old_line=='>');
        leq = find(old_line=='<');
        new_line = [old_line(1:geq(1)),new_eq,old_line(leq(2):end)];
    end
end