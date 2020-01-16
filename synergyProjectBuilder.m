file_dir = fileparts(mfilename('fullpath'));
proj_file = [fileparts(fileparts(mfilename('fullpath'))),'\SynergyControl\Animatlab\SynergyWalking\SynergyWalking20200109.aproj'];
revised_file = strcat(proj_file(1:end-6),'_fake_postmuscle.aproj');

fid = fopen(proj_file);
original_text = textscan(fid,'%s','delimiter','\n');
fclose(fid);
original_text = [original_text{:}];
muscleIDs = find(contains(original_text,'<PartType>AnimatGUI.DataObjects.Physical.Bodies.LinearHillMuscle</PartType>'))-2;

nsys = CanvasModel;
neurpos = [];
%Build motor neurons first
for ii = 1:length(W)
    neurpos = [(ii)*25.90 250];
    nsys.addItem('n', neurpos, [1000 1000])
    nsys.addMuscle([lower(original_text{muscleIDs(ii)-1}(7:end-7)),'-neural'],original_text{muscleIDs(ii)}(5:end-5),neurpos+[-25.9 150])
    create_adapter_link(nsys,ii,ii,ii,neurpos+[-25.9 50])
end

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
    synpos = [(ii-1)*25+(ii)*syngap 122.25];
    nsys.addItem('n', synpos, [1000 1000]);
    colordec = rgb2anim(cm(ii,:));
    nsys.neuron_objects(38+ii).color = colordec;
    for jj = 1:length(W)
        if W(jj,ii) ~= 0 
            nsys.addLink(nsys.neuron_objects(ii+length(W)),nsys.neuron_objects(jj),'SignalTransmission1')
            nsys.createSynapseType({['Syn-',num2str(ii+38),'-',num2str(jj)],'delE',194,'k',relW(jj,ii)})
            numLinks = size(nsys.link_objects,1);
            numSyns = size(nsys.synapse_types,1);
            nsys.link_objects(numLinks).synaptictype = nsys.synapse_types(numSyns).name;
        end
    end
    nsys.addStimulus(nsys.neuron_objects(38+ii))
    nsys.stimulus_objects(ii).starttime = 0;
    nsys.stimulus_objects(ii).endtime = 10;
    if isfile("G:\My Drive\Rat\SynergyControl\Data\h_equations.mat")
        load("G:\My Drive\Rat\SynergyControl\Data\h_equations.mat")
        nsys.stimulus_objects(ii).eq = equations{ii};
    else
        nsys.stimulus_objects(ii).eq = generate_synergy_eq(bigH(ii,:));
        equations{ii} = nsys.stimulus_objects(ii).eq;
        save("G:\My Drive\Rat\SynergyControl\Data\h_equations.mat",'equations')
    end
    nsys.addDTaxes(nsys.datatool_objects(1),nsys.neuron_objects(38+ii))
end

nsys.create_animatlab_project(proj_file);

function create_adapter_link(nsys,adInd,neurInd,muscInd,pos)
    nsys.addAdapter(nsys.neuron_objects(neurInd),nsys.muscle_objects(muscInd),pos)
    nsys.addLink(nsys.neuron_objects(neurInd),nsys.adapter_objects(adInd),'adapter')
    nsys.addLink(nsys.adapter_objects(adInd),nsys.muscle_objects(muscInd),'adapter')
end

function equation = generate_synergy_eq(bigH)
    dt = .00054;
    time = (99*dt:dt:10.01-10*dt)';
    
    bigH = interpolate_for_time(time,bigH);
    
    coeffs = cell(24,2);
    
    % Create a sum of sines equation for the joint angle waveforms
    fitresult = sumsines8Fit(time, bigH,8);
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