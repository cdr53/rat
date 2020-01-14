function obj = design_synergy(proj_file)

sour_folder = 'C:\Program Files (x86)\NeuroRobotic Technologies\AnimatLab\bin';

full_run = 1;
minimum_effort_run = 0;
isohip_run = 0;

if nargin < 1
    proj_file = [fileparts(mfilename('fullpath')),'\Animatlab\','IndividualMuscleStim20191114.asim'];
end

%proj_file = 'G:\My Drive\Rat\SynergyControl\Animatlab\SynergyWalking\SynergyWalking20200109_Standalone.asim';

if ispc
    if full_run
        % Define Animatlab .asim file to run
        %proj_file = [fileparts(mfilename('fullpath')),'\Animatlab\','IndividualMuscleStim20190429_mod_Standalone.asim'];
        % Run .asim file with system
        executable = ['"',sour_folder,'\AnimatSimulator" "',proj_file,'"'];
        [status, message] = system(executable);
        if status
            error(message)
            return
        end
        % Post-process data
        data_struct = importdata('Animatlab\JointMotion.txt');
        col_head_slice = data_struct.colheaders;
        joint_profile = data_struct.data(:,2:5);
            % Determine which columns correspond to which joint. Animatlab doesn't necessarily follow a hip->knee->ankle convention
            knee = joint_profile(:,2);
            ankle = joint_profile(:,3);
            hip = joint_profile(:,4);
        joint_profile = [joint_profile(:,1),hip,knee,ankle];
        % Prepare object instantiation variables
        joints = {[],'LH_HipZ','LH_Knee','LH_AnkleZ'};
        bodies = {'Pelvis','LH_Femur','LH_Tibia','LH_Foot'};
        % Joint Limit: hard coded values are the default. While experimenting with different waveforms, I disengaged the hard limits
        % in exchange for unbounded motion.
        % joint_limits = [-60,7.4;-50,60;-80,25]*pi/180;
        joint_limits = [min(joint_profile(:,2:end))' max(joint_profile(:,2:end))'];
        %Data_file = 'G:\My Drive\Rat\Rat Model Development\AllTraining.mat';
        obj = FullLeg(proj_file,joints,bodies,joint_limits,joint_profile);
    elseif isohip_run
        proj_file = 'C:\Users\Fletcher\Documents\AnimatLab\FullMuscleLeg20190326\FullMuscleLeg20190326.aproj';
        joints = {[],'LH_HipZ'};
        bodies = {'Pelvis','LH_Femur'};
        joint_limits = [-60,7.4]*pi/180;
        Data_file = 'G:\My Drive\Rat\Rat Model Development\AllTraining.mat';
        obj = IsoHip(proj_file,joints,bodies,joint_limits);
    elseif minimum_effort_run
        proj_file = 'C:\Users\Fletcher\Documents\AnimatLab\MinimumEffortModel\MinimumEffortModel_Standalone.asim';
        joints = {[],'LH_HipZ'};
        bodies = {'Pelvis','LH_Femur'};
        joint_limits = [-60,50]*pi/180;
        Data_file = 'G:\My Drive\Rat\Rat Model Development\AllTraining.mat';
    end
elseif ismac
    proj_file = '/Volumes/GoogleDrive/My Drive/Germany 2018/FullMuscleLeg20180906/FullMuscleLeg20180906.aproj';
    joints = {[],'LH_HipZ','LH_Knee','LH_AnkleZ'};
    bodies = {'Pelvis','LH_Femur','LH_Tibia','LH_Foot'};
    joint_limits = [-60,50;-80,60;-80,25]*pi/180;
    Data_file = '/Volumes/GoogleDrive/My Drive/Rat/Rat Model Development/AllTraining.mat';
else
    disp('Wait, pick which sim you are doing')
    keyboard
end

clearvars ankle bodies col_head_slice data_struct executable full_run hip isohip_run joint_limits joint_profile joints knee message minimum_effort_run ...
    proj_file sour_folder status
end