function parameter_injector(fPath,params)
    % For an input project path, update the muscle parameters
    % Input: fPath: Project path to update
    % Input: params: optional cell array with pre-assembled values
    
    %fPath = [pwd,'\Animatlab\SynergyWalking\SynergyWalking20200109_ptest.aproj'];
    
    if ~contains(fPath,'.aproj')
        error('fPath must point to a project file. The input path does not have an .aproj extension.\n')
    end
    
    project_file = importdata(fPath);
    muscle_addresses = contains(project_file,'<Type>LinearHillMuscle</Type>');
    muscle_indices = find(muscle_addresses)-2;
    nummusc = length(muscle_indices);

    if nargin == 1
        params = cell(nummusc,4);
        for ii = 1:nummusc
            mInd = muscle_indices(ii);
            rInd = find(contains(project_file(mInd:end),'<RestingLength'),1,'first')-1;
            fInd = find(contains(project_file(mInd:end),'<MaximumTension'),1,'first')-1;
%             Lr = obj.musc_obj{ii}.RestingLength;
%             Fo = obj.musc_obj{ii}.max_force;
            Lr = number_injector(project_file{mInd+rInd},[]);
            Fo = number_injector(project_file{mInd+fInd},[]);
            params{ii,1} = cell2mat(extractBetween(lower(project_file{mInd}),'<name>','</name>'));
            xx = equilsolver_func(Lr,Fo);
            params{ii,2} = xx(1);
            params{ii,3} = xx(2);
            % An internal debate: is the third column Am vals or the maximum force?
            %params{ii,4} = xx(3);
            params{ii,4} = 1.2*Fo; %ST_max values, should be slight larger than Fmax to ensure Am values are possible.
            [params{:,5}] = deal(661.663);
            [params{:,6}] = deal(0);
        end
    elseif nargin == 0
        error('Please include a path to an Animatlab project.\n')
    end
    
    lr = load([pwd,'\Data\neutral_lengths.mat'],'lr');
    params = import_johnson_data(params,lr);
        
    %These are the parameters represented in the .aproj file. We will iterate through them to udate values. The order of these are important and correspond to
    %the column order in the params variable
    parameter_terms = {'<Kse Value';...
                       '<Kpe Value';...
                       '<B Value';...
                       '<C Value';...
                       '<D Value';...
                       '<LowerLimitScale';...
                       '<RestingLength';...
                       '<UpperLimitScale';...
                       '<Lwidth';...
                       '<MaximumTension'};
                    
    for i=1:nummusc
        muscind = muscle_indices(i);
        for j = 1:length(parameter_terms)
            if strcmp(parameter_terms(j),'<LowerLimitScale') || strcmp(parameter_terms(j),'<UpperLimitScale')
                partemp = find(contains(project_file(muscind:end),parameter_terms{j}))+muscind-1;
                par_ind = partemp(2);
            else
                par_ind = find(contains(project_file(muscind:end),parameter_terms{j}),1)+muscind-1;
            end
%                 initpar = project_file{par_ind};
            project_file{par_ind} = number_injector(project_file{par_ind},round(params{i,j+1},2));
%                fprintf('From %s to %s\n\n',initpar,project_file{par_ind})
        end
    end
    
    [~,projName] = fileparts(fPath);
    carry_on = input(['You are about to overwrite the Animatlab project file (',projName,') you''re using with new parameters.\n'...
                        'This could permanently ruin your project file if there are errors.\n'...
                        'If this is what you want to do, type YES. Otherwise, the program will not overwrite.\n'],'s');
                    
    if strcmp(carry_on,'YES')
        % Record values into a spreadsheet
        spreadsheet = [pwd,'\ParameterCalculation\MuscleParameters_20190513.xlsx'];
        params2write = sortrows(params);
        xlswrite(spreadsheet,params2write(:,2:end),'G2:P39');
        % Update the provided project file with the new parameters
        fileID = fopen(fPath,'w');
        formatSpec = '%s\n';
        nrows = size(project_file);
        for row = 1:nrows
            fprintf(fileID,formatSpec,project_file{row,:});
        end
        fclose(fileID);
    end
    
    function new_line = number_injector(old_line,number)
        quote_inds = find(old_line=='"');
        value = num2str(number);
        if isempty(number) && nargin == 2
            new_line = str2double(cell2mat(extractBetween(old_line,'Actual="','"/>')));
            return
        end
        if contains(old_line,'None')
            actual = value;
        else
            modifier = old_line(quote_inds(3)+1:quote_inds(4)-1);
            switch modifier
                case 'milli'
                    actual = num2str(number/1000);
                case 'centi'
                    actual = num2str(number/100);
            end
        end
        new_line = [old_line(1:quote_inds(1)),value,old_line(quote_inds(2):quote_inds(5)),actual,old_line(quote_inds(6):end)];
    end

    function revised_params = import_johnson_data(params,neutral_lengths)
        % Expand the input parameter array to include information from Johnson 2011 parameters and calculated LT limits/values
        % Input: params: nx6 parameter cell array containing [muscle name, Kse, Kpe, Am, steepness, y offset(?)]
        johnson_excel = [pwd,'\ParameterCalculation\MuscleParameters_20190513.xlsx'];
        C = readcell(johnson_excel);
        [rows,cols] = size(params);
        revised_params = cell(rows,cols+5);
        revised_params(1:rows,1:cols) = params;
        
        %Generate a list of the Johnson data muscle names. These aren't in the same order as the project file
        johnsonnamelist = cell(length(C),1);
        for q = 2:length(C)
            temp = lower(C{q,1});
            johnsonnamelist{q,1} = temp(~isspace(temp));
        end
        
        %Check if the imported resting lengths are in mm or centimeters. Want them in centimeters
        resting_lengths = cell2mat(C(2:end,4));
        if mean(resting_lengths)/10 > 1
            scale = 10;
        else
            scale = 1;
        end
        
        for p = 1:length(revised_params)
            %Determine which Johnson muscle to load
            johnsonind = find(contains(johnsonnamelist(2:end),revised_params{p,1}(4:end)),1)+1;
            % Set l_rest to the neutral length
            l_rest = neutral_lengths.lr{p,2}*100;
            % Pull in the optimal force information from the spreadsheet
            f_o = C{johnsonind,6};
            % Lower length limit of the LT curve
            revised_params{p,cols+1} = .5*l_rest;
            % L rest
            revised_params{p,cols+2} = l_rest;
            % Upper length limit of the LT curve
            revised_params{p,cols+3} = 1.5*l_rest;
            % L width
            revised_params{p,cols+4} = .5*l_rest;
            % Maximum tension
            revised_params{p,cols+5} = 1.8*f_o;
        end
    end
end