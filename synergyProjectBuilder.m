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
cm = cm(1:3:end,:);
cm = round(cm*255);
syngap = (958.5-size(W,2)*(25))/(size(W,2)+1);
%Build synergy neurons and connections
for ii = 1:size(W,2)
   synpos = [(ii-1)*25+(ii)*syngap 122.25];
   nsys.addItem('n', synpos, [1000 1000]);
   colordec = cm(ii,1)*255*255+cm(ii,2)*255+cm(ii,3);
   nsys.neuron_objects(38+ii).color = colordec;
   for jj = 1:length(W)
        if W(jj,ii) ~= 0 
            nsys.addLink(nsys.neuron_objects(ii+length(W)),nsys.neuron_objects(jj),'SignalTransmission1')
        end
   end
   nsys.addStimulus(nsys.neuron_objects(38+ii))
   nsys.stimulus_objects(ii).starttime = randi([0 5],1);
   nsys.stimulus_objects(ii).endtime = randi([6 10],1);
   nsys.stimulus_objects(ii).magnitude = randi([5 15],1);
end

nsys.create_animatlab_project(proj_file);

function create_adapter_link(nsys,adInd,neurInd,muscInd,pos)
    nsys.addAdapter(nsys.neuron_objects(neurInd),nsys.muscle_objects(muscInd),pos)
    nsys.addLink(nsys.neuron_objects(neurInd),nsys.adapter_objects(adInd),'adapter')
    nsys.addLink(nsys.adapter_objects(adInd),nsys.muscle_objects(muscInd),'adapter')
end