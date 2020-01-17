nsys = CanvasModel();
muscle_out = scrape_project_for_musc_info(original_text);

nsys.addItem('n',[100 50], [1000 1000])
nsys.addMuscle(muscle_out{1,2},muscle_out{1,3},[100 150])
nsys.addLink(nsys.neuron_objects(1),nsys.muscle_objects(1),'adapter')
nsys.addStimulus(nsys.neuron_objects(1))
nsys.stimulus_objects(1).starttime = 1;
nsys.addDatatool('simpleDT')
nsys.addDTaxes(nsys.datatool_objects(1),nsys.muscle_objects(1),'Tension')
nsys.addDTaxes(nsys.datatool_objects(1),nsys.muscle_objects(1),'MembraneVoltage')

nsys.create_animatlab_project;

function muscle_out = scrape_project_for_musc_info(input_text)
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