function muscle_info = scrapeFileForMuscleInfo(fPath)
    % Scrape text for muscle information
    % Input: input_text: cell array of text from an Animatlab project
    % Output: muscle_out: cell array containing muscle indices, muscle names, and muscle ID's
    original_text = importdata(fPath);
    muscleInds = find(contains(original_text,'<PartType>AnimatGUI.DataObjects.Physical.Bodies.LinearHillMuscle</PartType>'))-2;
    if isempty(muscleInds)
        error('No muscles in input text')
    end
    muscle_info = cell(length(muscleInds),3);
    for ii = 1:length(muscleInds)
        muscle_info{ii,1} = muscleInds(ii);
        muscle_info{ii,2} = lower(original_text{muscleInds(ii)-1}(7:end-7));
        muscle_info{ii,3} = original_text{muscleInds(ii)}(5:end-5);
    end
end