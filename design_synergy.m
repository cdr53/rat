function [obj,sim_file,joints,bodies,joint_limits,joint_profile] = design_synergy(sim_file)

full_run = 1;
minimum_effort_run = 0;
isohip_run = 0;

if nargin < 1
    sim_file = [fileparts(mfilename('fullpath')),'\Animatlab\SynergyWalking\SynergyWalking20200109_Standalone.asim'];
end

if isstring(sim_file)
    sim_file = char(sim_file);
end

if ispc
    if full_run
        % Define Animatlab .asim file to run
        % sim_file = [fileparts(mfilename('fullpath')),'\Animatlab\','IndividualMuscleStim20190429_mod_Standalone.asim'];
        % Run .asim file with system
        sdata = processSimData(sim_file);
        aForms = {'JointMotion';'PassiveTension'};
        for ii = 1:length(aForms)
            tempInd = find(contains({sdata.name},aForms{ii}));
            data = sdata(tempInd).data;
            %data_struct = importdata([fileparts(sim_file),'\',aForms{ii},'.txt']);
            %col_head_slice = data_struct.colheaders;
            col_head_slice = sdata(tempInd).data_cols;
            switch aForms{ii}
                case 'JointMotion'
                        hipInd = find(contains(col_head_slice,'Hip'),1,'first');
                        kneeInd = find(contains(col_head_slice,'Knee'),1,'first');
                        ankleInd = find(contains(col_head_slice,'Ankle'),1,'first');
                    joint_profile = sdata(tempInd).data;
                        % Determine which columns correspond to which joint. Animatlab doesn't necessarily follow a hip->knee->ankle convention
                        hip = joint_profile(:,hipInd);
                        knee = joint_profile(:,kneeInd);
                        ankle = joint_profile(:,ankleInd);
                    joint_profile = [sdata(tempInd).time,hip,knee,ankle];
                case 'PassiveTension'
                    passive_tension = cell(length(col_head_slice),2);
                    for jj = 1:length(col_head_slice)
                        passive_tension{jj,1} = col_head_slice{jj};
                        passive_tension{jj,2} = data(:,jj);
                    end
            end
        end
        % Prepare object instantiation variables
        joints = {[],'LH_HipZ','LH_Knee','LH_AnkleZ'};
        bodies = {'Pelvis','LH_Femur','LH_Tibia','LH_Foot'};
        % Joint Limit: hard coded values are the default. While experimenting with different waveforms, I disengaged the hard limits
        % in exchange for unbounded motion.
        % joint_limits = [-60,7.4;-50,60;-80,25]*pi/180;
        joint_limits = [min(joint_profile(:,2:end))' max(joint_profile(:,2:end))'];
        %Data_file = 'G:\My Drive\Rat\Rat Model Development\AllTraining.mat';
        obj = FullLeg(sim_file,joints,bodies,joint_limits,joint_profile,passive_tension);
    elseif isohip_run
        sim_file = 'C:\Users\Fletcher\Documents\AnimatLab\FullMuscleLeg20190326\FullMuscleLeg20190326.aproj';
        joints = {[],'LH_HipZ'};
        bodies = {'Pelvis','LH_Femur'};
        joint_limits = [-60,7.4]*pi/180;
        Data_file = 'G:\My Drive\Rat\Rat Model Development\AllTraining.mat';
        obj = IsoHip(sim_file,joints,bodies,joint_limits);
    elseif minimum_effort_run
        sim_file = 'C:\Users\Fletcher\Documents\AnimatLab\MinimumEffortModel\MinimumEffortModel_Standalone.asim';
        joints = {[],'LH_HipZ'};
        bodies = {'Pelvis','LH_Femur'};
        joint_limits = [-60,50]*pi/180;
        Data_file = 'G:\My Drive\Rat\Rat Model Development\AllTraining.mat';
    end
elseif ismac
    sim_file = '/Volumes/GoogleDrive/My Drive/Germany 2018/FullMuscleLeg20180906/FullMuscleLeg20180906.aproj';
    joints = {[],'LH_HipZ','LH_Knee','LH_AnkleZ'};
    bodies = {'Pelvis','LH_Femur','LH_Tibia','LH_Foot'};
    joint_limits = [-60,50;-80,60;-80,25]*pi/180;
    Data_file = '/Volumes/GoogleDrive/My Drive/Rat/Rat Model Development/AllTraining.mat';
else
    disp('Wait, pick which sim you are doing')
    keyboard
end

end