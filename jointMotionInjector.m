function [obj] = jointMotionInjector(jointAngles,to_plot)
    % Inject joint angle waveform array and generate a FullLeg object from the simulation results
    % Input: jointAngles: nx3 sagittal-plane joint angle waveforms with columns formatted as hip-knee-ankle
    % Input: to_plot: boolean for determining whether to plot results
    % Output: FullLeg object with the associated joint angles
    
    if nargin == 0
        load([fileparts(mfilename('fullpath')),'\Data\processedHindlimbAngles.mat'],'BackMean','BackRaw','completeWaves')
        % jointAngles = [BackRaw(:,3,1)-98,BackRaw(:,3,2)-90,BackRaw(:,3,3)-116];
        jointAngles = [completeWaves(:,1,1)-98,completeWaves(:,1,2)-90,completeWaves(:,1,3)-116];
        to_plot = 0;
    else
        if ~any(size(jointAngles)==3)
            error('Please format input jointAngles as an nx3 array with joint angles formatted as hip-knee-ankle')
        else
            if size(jointAngles,2) ~= 3
                jointAngles = jointAngles';
            end
        end
        if nargin == 2 && ~ismember(to_plot,[0 1])
            warning('to_plot must be 0 or 1, not %.0f. Setting to 0.',to_plot)
        end
    end
    
    % Which .asim file to modify?
    %simfilepath = [fileparts(mfilename('fullpath')),'\Animatlab\IndividualMuscleStim20191114.asim'];
    simfilepath = [pwd,'\Animatlab\SynergyWalking\SynergyWalking20200109_Standalone.asim'];
    meshMatch(simfilepath)
    file_contents = importdata(simfilepath);
    
    % Before moving forward, check that the simulation file contains the proper MotorPosition stimuli that are necessary to simulate walking
    file_contents = disableAllStims(file_contents);
    reqNames = {'<Name>Hipwalking';'<Name>Kneewalking';'<Name>xAnklewalking';'JointMotion.txt'};
    for ii = 1:length(reqNames)
        file_contents = checkFor(file_contents,reqNames{ii});
    end
    
    if mean(mean(abs(jointAngles)))>2
        jointAngles = [jointAngles;jointAngles;jointAngles].*(pi/180);
    else
        jointAngles = [jointAngles;jointAngles;jointAngles];
    end
    
    % From the .asim file, extract time data so that we can match the input joint waveforms to the right time vector
    simtime_ind = find(contains(file_contents,'<APIFile/>'))+1;
    simTime = time_extractor(file_contents{simtime_ind});
    dt = time_extractor(file_contents{contains(file_contents,'<PhysicsTimeStep>')});
    time = (99*dt:dt:simTime-10*dt)';
    
    jointAnglesBig = interpolate_for_time(time,jointAngles);

    %Preallocate for speed
    equations = cell(3,1);
    
    parfor j = 1:3
        % Create a sum of sines equation for the joint angle waveforms
        %fitresult = sumsines8Fit(time, jointAnglesBig(:,j),8);
        [fitresult] = sumsinesFit(time, jointAnglesBig(:,j));
        % Coeffs are the a, b, and c values in the equation a*sin(b*t+c)
        coeffs = [coeffnames(fitresult),num2cell(coeffvalues(fitresult)')];
        % Equations are in the format necessary for integration into Animatlab's .asim filetype
        equations{j} = sum_of_sines_maker(coeffs,0);
    end

    % Injects new joint angles into motor stimuli at each joint
    inject_joint_waveforms(simfilepath,equations,round(simTime));
    
    % Update the actual simulation file now that edits have been made
    overwriteSimFile(simfilepath,file_contents);
    
    obj = design_synergy(simfilepath);
    
    if to_plot
        ymax = 1.1*max(max([jointAnglesBig;obj.theta_motion]));
        ymin = 1.1*min(min([jointAnglesBig;obj.theta_motion]));
        subplot(2,1,1)
        plot(time,jointAnglesBig,'LineWidth',2)
        xlabel('Input waveforms')
        ylim([ymin ymax])
        xlim([0 time(end)])
        subplot(2,1,2)
        plot(obj.theta_motion_time,obj.theta_motion,'LineWidth',2)
        xlabel('Sim output')
        ylim([ymin ymax])
        xlim([0 time(end)])
    end

    function maxTime = time_extractor(old_line)
        geq = find(old_line=='>');
        leq = find(old_line=='<');
        maxTime = str2double(old_line(geq(1)+1:leq(2)-1));
    end
    
    function waveformsBig = interpolate_for_time(time,waveforms)
        % Interpolate the undersampled input to match the required time vector
        m = length(time);
        n = length(waveforms);
        if m ~= n
            waveformsBig = interp1(1:n,waveforms,linspace(1,n,m));
        end

        avgblocks = floor(.01*length(waveformsBig));
        coeffblocks = ones(1,avgblocks)/avgblocks;
        for jj=1:3
            for i=1:2
                waveformsBig(:,jj) = filtfilt(coeffblocks,1,waveformsBig(:,jj));
            end
        end
    end

    function fileContents = checkFor(fileContents,reqName)
        stimSearch = contains(fileContents,reqName);
        if ~any(stimSearch)
            error('jointMotionInjector: Input simulation file does not contain an item called %s.',reqName)
        else
            enInd = find(stimSearch)+2;
            if contains(fileContents{enInd},'False') && ~contains(reqName,'.txt')
                fileContents{enInd} = '<Enabled>True</Enabled>';
            end
        end
    end

    function fileContents = disableAllStims(fileContents)
       stimInds = find(contains(fileContents,'<Stimulus>'));
       for tt = 1:length(stimInds)
            sI = stimInds(tt);
            stimEn = find(contains(fileContents(sI:end),'<Enabled>'),1,'first')-1;
            fileContents{sI+stimEn} = '<Enabled>False</Enabled>';
       end
    end

    function overwriteSimFile(fPath,docContents)
        fileID = fopen(fPath,'w');
        formatSpec = '%s\n';
        nrows = size(docContents);
        for row = 1:nrows
            fprintf(fileID,formatSpec,docContents{row,:});
        end
        fclose(fileID);
    end
end