file_dir = fileparts(mfilename('fullpath'));
proj_file = [pwd,'\Animatlab\SynergyWalking\SynergyWalking20200109.aproj'];
meshMatch(proj_file)
revised_file = strcat(proj_file(1:end-6),'_fake.aproj');
[projDir,projName,ext] = fileparts(revised_file);
disp(['Starting to build Animatlab project ',projName,ext])
delete([projDir,'\Trace*'])

original_text = importdata(proj_file);
muscleIDs = find(contains(original_text,'<PartType>AnimatGUI.DataObjects.Physical.Bodies.LinearHillMuscle</PartType>'))-2;
muscle_out = scrape_project_for_musc_info(original_text);
numMuscles = length(muscleIDs);

nsys = CanvasModel;
neurpos = [];

%Build a line of motor neurons and muscles first
for ii = 1:numMuscles
    neurpos = [(ii)*25.90 250];
    nsys.addItem('n', neurpos, [1000 1000])
    nsys.addMuscle([muscle_out{ii,2},'-neural'],muscle_out{ii,3},neurpos+[-25.9 150])
    nsys.addLink(nsys.neuron_objects(ii),nsys.muscle_objects(ii),'adapter')
end

% Generate color palette for synergy nodes
cm = jet(3*size(W,2));
cm = round(cm(3:3:end,:)*255);

syngap = (958.5-size(W,2)*(25))/(size(W,2)+1);

relW = W./max(W);
bigH = (H'.*max(W))';

gsyn = arrayfun(@(x) (x*20)/(194-x*20),relW);
equations = cell(size(W,2),1);
nsys.addDatatool('SynergyStim');

%Build synergy neurons and connections
for ii = 1:size(W,2)
    synpos = [(ii-1)*25+(ii)*syngap 100];
    nsys.addItem('n', synpos, [1000 1000]);
    colordec = rgb2anim(cm(ii,:));
    nsys.neuron_objects(numMuscles+ii).color = colordec;
    for jj = 1:length(W)
        if W(jj,ii) ~= 0 
            nsys.addLink(nsys.neuron_objects(ii+length(W)),nsys.neuron_objects(jj),'SignalTransmission1')
            nsys.createSynapseType({['Syn-',num2str(ii+numMuscles),'-',num2str(jj)],'delE',194,'k',relW(jj,ii)})
            %gsyn = (20*W(jj,ii))/(size(W,2)*194);
            %nsys.createSynapseType({['Syn-',num2str(ii+numMuscles),'-',num2str(jj)],'delE',194,'max_syn_cond',gsyn})
            numLinks = size(nsys.link_objects,1);
            numSyns = size(nsys.synapse_types,1);
%             nsys.synapse_types(numSyns).presyn_thresh = -120;
%             nsys.synapse_types(numSyns).presyn_sat = 0;
            nsys.link_objects(numLinks).synaptictype = nsys.synapse_types(numSyns).name;
        end
    end
    nsys.addStimulus(nsys.neuron_objects(numMuscles+ii))
    nsys.stimulus_objects(ii).starttime = 0;
    nsys.stimulus_objects(ii).endtime = 10;
    if isfile([file_dir,'\Data\h_equations.mat'])
        load([file_dir,'\Data\h_equations.mat'])
        nsys.stimulus_objects(ii).eq = equations{ii};
    else
        sinewavetest = 0;
        if sinewavetest
            tt = 0:.001:2*pi;
            eqns = {2.5*sin(tt)+2.5;...
                    10*sin(2*tt)+10;...
                    5*cos(3*tt)+5;...
                    2*cos(2*tt)+14;...
                    4*sin(4*tt)+14};
            nsys.stimulus_objects(ii).eq = generate_synergy_eq(eqns{ii});
        else
            nsys.stimulus_objects(ii).eq = generate_synergy_eq(bigH(ii,:));
        end
        %nsys.stimulus_objects(ii).eq = generate_synergy_eq(bigH(ii,:));
        equations{ii} = nsys.stimulus_objects(ii).eq;
        if  ii == size(W,2)
            save([file_dir,'\Data\h_equations.mat'],'equations')
        end
    end
    nsys.addDTaxes(nsys.datatool_objects(1),nsys.neuron_objects(numMuscles+ii),'MembraneVoltage')
end

% Build datatool viewers for high action muscles to provide insight into motorneuron and muscle activity
[muscForces,muscInds] = sortrows(max(forces)',1,'descend');
muscles2check = muscInds(1:5);
numDTs = size(nsys.datatool_objects,1);
nsys.addDatatool('KeyMNs')
nsys.addDatatool('KeyMuscles')
nsys.addDatatool('KeyMuscTen')
% For a given muscle, find that muscle's adapter information and then find the *neuron* that's feeding that adapter
for ii = 1:length(muscles2check)
    adInd = find(contains({nsys.adapter_objects(:).destination_node_ID},nsys.muscle_objects(muscles2check(ii)).ID));
    nInd = find(contains({nsys.neuron_objects(:).ID},nsys.adapter_objects(adInd).origin_node_ID));
    nsys.addDTaxes(nsys.datatool_objects(numDTs+1),nsys.neuron_objects(nInd),'MembraneVoltage')
    nsys.addDTaxes(nsys.datatool_objects(numDTs+2),nsys.muscle_objects(muscles2check(ii)),'MembraneVoltage')
    nsys.addDTaxes(nsys.datatool_objects(numDTs+3),nsys.muscle_objects(muscles2check(ii)),'Tension')
end

nsys.create_animatlab_project(proj_file);
disp(['Animatlab project file ',projName,ext,' created.'])

function equation = generate_synergy_eq(bigH)
    dt = .00054;
    time = (99*dt:dt:10.01-10*dt)';
    
    bigH = interpolate_for_time(time,bigH);
    
    % Create a sum of sines equation for the joint angle waveforms
    fitresult = sumsinesFit(time, bigH,8);
    % Coeffs are the a, b, and c values in the equation a*sin(b*t+c)
    coeffs = [coeffnames(fitresult),num2cell(coeffvalues(fitresult)')];
    % Equations are in the format necessary for integration into Animatlab's .asim filetype
    equation = sum_of_sines_maker(coeffs,1);
end

function waveformsBig = interpolate_for_time(time,waveforms)
    % Interpolate the undersampled input to match the required time vector
    waveforms = waveforms';
    m = length(time);
    n = length(waveforms);
    if m ~= n
        waveformsBig = interp1(1:n,waveforms,linspace(1,n,m));
    end

    avgblocks = floor(.01*length(waveformsBig));
    coeffblocks = ones(1,avgblocks)/avgblocks;
    for i=1:2
        waveformsBig = filtfilt(coeffblocks,1,waveformsBig);
    end
    waveformsBig = waveformsBig';
end

function muscle_out = scrape_project_for_musc_info(input_text)
    % Scrape text for muscle information
    % Input: input_text: cell array of text from an Animatlab project
    % Output: muscle_out: cell array containing muscle indices, muscle names, and muscle ID's
    muscleInds = find(contains(input_text,'<PartType>AnimatGUI.DataObjects.Physical.Bodies.LinearHillMuscle</PartType>'))-2;
    if isempty(muscleInds)
        error('No muscles in input text')
    end
    muscle_out = cell(length(muscleInds),3);
    for ii = 1:length(muscleInds)
        muscle_out{ii,1} = muscleInds(ii);
        muscle_out{ii,2} = lower(input_text{muscleInds(ii)-1}(7:end-7));
        muscle_out{ii,3} = input_text{muscleInds(ii)}(5:end-5);
    end
end