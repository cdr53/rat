nsys = CanvasModel();

% for ii = 1:2
%     neurpos = [(ii)*100 250];
%     nsys.addItem('n', neurpos, [1000 1000])
%     nsys.addMuscle([lower(original_text{muscleIDs(ii)-1}(7:end-7)),'-neural'],original_text{muscleIDs(ii)}(5:end-5),neurpos+[-25.9 150])
% end
% 
% nsys.addLink(nsys.neuron_objects(1),nsys.neuron_objects(2),'SignalTransmission1')
% 
% % nsys.addAdapter(nsys.neuron_objects(2),nsys.muscle_objects(2),[25.9 300])
% % nsys.addLink(nsys.neuron_objects(2),nsys.adapter_objects(1),'adapter')
% % nsys.addLink(nsys.adapter_objects(1),nsys.muscle_objects(2),'adapter')
% % 
% nsys.addAdapter(nsys.neuron_objects(1),nsys.muscle_objects(1),[100 300])
% nsys.addLink(nsys.neuron_objects(1),nsys.adapter_objects(1),'adapter')
% nsys.addLink(nsys.adapter_objects(1),nsys.muscle_objects(1),'adapter')
% % 
% nsys.create_animatlab_project;

nsys = CanvasModel();
nsys.addItem()
nsys.addStimulus(nsys.neuron_objects(1))