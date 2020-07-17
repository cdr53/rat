simPath = 'G:\My Drive\Rat\SynergyControl\Animatlab\SynergyWalking\SynergyWalking_EMGwalking_Standalone.asim';
projPath = 'G:\My Drive\Rat\SynergyControl\Animatlab\SynergyWalking\SynergyWalking_EMGwalking.aproj';
load('G:\My Drive\Rat\SynergyControl\Data\emgTestSet.mat');

% Process data locations 
    docDir = fileparts(simPath);
    fclose('all');
    delete([docDir,'\Trace*'])
    delete([pwd,'\Data\InjectedCurrent\*'])

% scrape input file for muscle info
    muscle_out = scrapeFileForMuscleInfo(projPath);
    numMuscles = length(muscle_out);    
    
nsys = CanvasModel;
activation = emg2activation(emgMns{1});
%activation(:,[1:7,9:12]) = 0;

for ii=1:numMuscles
    % make neuron
        neurpos = [(ii)*25.90 250];
        nsys.addItem('n', neurpos, [1000 1000]);
    % make muscle
        nsys.addMuscle([muscle_out{ii,2},'-neural'],muscle_out{ii,3},neurpos+[-25.9 150]);
    % make link neuron and muscle
        nsys.addLink(nsys.neuron_objects(ii),nsys.muscle_objects(ii),'adapter')
    % make direct current stim
        nsys.addStimulus(nsys.neuron_objects(ii),'dc')
    % define stimulus waveform
        emgColumn = find(contains(emgMns{2}(2,:),muscle_out{ii,2}));
        dataWave = [activation(:,emgColumn);activation(:,emgColumn);activation(:,emgColumn)]*20;
        [outData,stimPath] = generate_direct_current_file(nsys,dataWave,nsys.stimulus_objects(ii).name);
        nsys.stimulus_objects(ii).current_wave = outData;
        nsys.stimulus_objects(ii).current_data_file = stimPath;
end

% build datatools
nsys.addDatatool({'KeyMNs','endtime',nsys.proj_params.simendtime-.01})
nsys.addDatatool({'KeyMuscleLen','endtime',nsys.proj_params.simendtime-.01})
nsys.addDatatool({'KeyMuscTen','endtime',nsys.proj_params.simendtime-.01})
nsys.addDatatool({'KeyMuscAct','endtime',nsys.proj_params.simendtime-.01})
nsys.addDatatool({'KeyMuscAl','endtime',nsys.proj_params.simendtime-.01})
% For a given muscle, find that muscle's adapter information and then find the *neuron* that's feeding that adapter
for ii = 1:numMuscles
    adInd = find(contains({nsys.adapter_objects(:).destination_node_ID},nsys.muscle_objects(ii).ID));
    nInd = find(contains({nsys.neuron_objects(:).ID},nsys.adapter_objects(adInd).origin_node_ID));
    nsys.addDTaxes('KeyMNs',nsys.neuron_objects(nInd),'MembraneVoltage')
    nsys.addDTaxes('KeyMuscleLen',nsys.muscle_objects(ii),'MuscleLength')
    nsys.addDTaxes('KeyMuscTen',nsys.muscle_objects(ii),'Tension')
    nsys.addDTaxes('KeyMuscAct',nsys.muscle_objects(ii),'Activation')
    nsys.addDTaxes('KeyMuscAl',nsys.muscle_objects(ii),'Tl')
end

cm = floor(jet(numMuscles)*255);
% recolor axis lines to make it easier to differentiate plot elements
for ii = 1:length(nsys.datatool_objects)
    for jj = 1:length(nsys.datatool_objects(ii).axes_objects)
        nsys.datatool_objects(ii).axes_objects(jj).linecolor = rgb2anim(cm(jj,:));
    end
end

% write object to proj and sim files
    nsys.create_animatlab_simulation(simPath);
    nsys.create_animatlab_project(projPath);