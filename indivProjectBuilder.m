% file_dir = fileparts(mfilename('fullpath'));
 proj_file = [pwd,'\Animatlab\SynergyWalking\SynergyWalking20200109.aproj'];
% meshMatch(proj_file);
% revised_file = strcat(proj_file(1:end-6),'_fake.aproj');
% [projDir,projName,ext] = fileparts(revised_file);
% disp(['Starting to build Animatlab project ',projName,ext])
% delete([projDir,'\Trace*'])
% 
% original_text = importdata(proj_file);
% muscleIDs = find(contains(original_text,'<PartType>AnimatGUI.DataObjects.Physical.Bodies.LinearHillMuscle</PartType>'))-2;
% muscle_out = scrape_project_for_musc_info(original_text);
% numMuscles = length(muscleIDs);

simPath = [pwd,'\Animatlab\SynergyWalking\muscleStim.asim'];
muscle_out = scrapeFileForMuscleInfo(simPath);
numMuscles = length(muscle_out);
[docDir,docName,docType] = fileparts(simPath);
%disp(['Starting to build Animatlab ',docType,' file "',docName,docType,'".'])
delete([docDir,'\Trace*'])

nsys = CanvasModel;
neurpos = [];
equations = cell(38,1);

%% The Time Debacle
%[beg,ennd] = obj.find_step_indices;
times = obj.theta_motion_time([obj.sampling_vector(1),obj.sampling_vector(end)]);
simTime = diff(times);
%simTime = max(obj.theta_motion_time);
nsys.proj_params.simendtime = simTime+.01;
nsys.proj_params.physicstimestep = 1000*(obj.dt_motion); % dt in ms

ci2 = movmean(current2inject',floor(length(current2inject)/3));
constvals = mean(ci2);

%Build a line of motor neurons and muscles first
for ii = 1:numMuscles
    neurpos = [(ii)*25.90 250];
    nsys.addItem('n', neurpos, [1000 1000])
    nsys.addMuscle([muscle_out{ii,2},'-neural'],muscle_out{ii,3},neurpos+[-25.9 150])
    nsys.addLink(nsys.neuron_objects(ii),nsys.muscle_objects(ii),'adapter')
    nsys.addStimulus(nsys.neuron_objects(ii))
    nsys.stimulus_objects(ii).starttime = 0;
    nsys.stimulus_objects(ii).endtime = simTime;
    if isfile([file_dir,'\Data\indiv_equations.mat'])
        load([file_dir,'\Data\indiv_equations.mat'])
        nsys.stimulus_objects(ii).eq = equations{ii};
    else
        %inputWave = mean(current2inject(ii,length(current2inject)*.1:length(current2inject)*.9));
        inputWave = current2inject(ii,:);
        nsys.stimulus_objects(ii).eq = generate_synergy_eq(constvals(ii),simTime,obj.dt_motion,0);
        %nsys.stimulus_objects(ii).eq = generate_synergy_eq(inputWave,simTime,obj.dt_motion,0);
        equations{ii} = nsys.stimulus_objects(ii).eq;
        if  ii == 38
            save([file_dir,'\Data\indiv_equations.mat'],'equations')
        end
    end
end

% Build datatool viewers for high action muscles to provide insight into motorneuron and muscle activity
[muscForces,muscInds] = sortrows(max(forces)',1,'descend');
%muscles2check = muscInds(1:5);
muscles2check = 1:38;
numDTs = size(nsys.datatool_objects,1);
nsys.addDatatool({'KeyMNs','endtime',nsys.proj_params.simendtime-.01})
nsys.addDatatool({'KeyMuscleLen','endtime',nsys.proj_params.simendtime-.01})
nsys.addDatatool({'KeyMuscTen','endtime',nsys.proj_params.simendtime-.01})
nsys.addDatatool({'KeyMuscAct','endtime',nsys.proj_params.simendtime-.01})
nsys.addDatatool({'KeyMuscAl','endtime',nsys.proj_params.simendtime-.01})
% For a given muscle, find that muscle's adapter information and then find the *neuron* that's feeding that adapter
for ii = 1:length(muscles2check)
    adInd = find(contains({nsys.adapter_objects(:).destination_node_ID},nsys.muscle_objects(muscles2check(ii)).ID));
    nInd = find(contains({nsys.neuron_objects(:).ID},nsys.adapter_objects(adInd).origin_node_ID));
    nsys.addDTaxes('KeyMNs',nsys.neuron_objects(nInd),'MembraneVoltage')
    nsys.addDTaxes('KeyMuscleLen',nsys.muscle_objects(muscles2check(ii)),'MuscleLength')
    nsys.addDTaxes('KeyMuscTen',nsys.muscle_objects(muscles2check(ii)),'Tension')
    nsys.addDTaxes('KeyMuscAct',nsys.muscle_objects(muscles2check(ii)),'Activation')
    nsys.addDTaxes('KeyMuscAl',nsys.muscle_objects(muscles2check(ii)),'Tl')
end

nsys.create_animatlab_simulation(simPath);
nsys.create_animatlab_project(proj_file);
%disp(['Animatlab ',docType,' file "',docName,docType,' created.'])

function equation = generate_synergy_eq(bigH,simTime,dt,projBool)
    %dt = .00054;
    time = (0:dt:simTime)';
    
    if sum(size(bigH)) == 2
        coeffs = cell(3,2);
        coeffs(1:3,1) = [{'a1'};{'b1'};{'c1'}];
        coeffs(1:3,2) = [{bigH}, {0}, {1.5708}];
        equation = sum_of_sines_maker(coeffs,1);
    else

        bigH = interpolate_for_time(time,bigH);

        % Create a sum of sines equation for the joint angle waveforms
        fitresult = sumsinesFit(time, bigH,8);
        % Coeffs are the a, b, and c values in the equation a*sin(b*t+c)
        coeffs = [coeffnames(fitresult),num2cell(coeffvalues(fitresult)')];
        % Equations are in the format necessary for integration into Animatlab's .asim filetype
        equation = sum_of_sines_maker(coeffs,projBool);
    end
end

function waveformsBig = interpolate_for_time(time,waveforms)
    % Interpolate the undersampled input to match the required time vector
    waveforms = waveforms';
    m = length(time);
    n = length(waveforms);
    if m ~= n
        waveformsBig = interp1(1:n,waveforms,linspace(1,n,m));
    else
        waveformsBig = waveforms';
    end

    avgblocks = floor(.01*length(waveformsBig));
    coeffblocks = ones(1,avgblocks)/avgblocks;
    for i=1:2
        waveformsBig = filtfilt(coeffblocks,1,waveformsBig);
    end
    waveformsBig = waveformsBig';
end