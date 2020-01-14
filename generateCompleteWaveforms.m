% Script for running waveFormCompleter onall three joints. Used for completing incomplete joint motion data

varNames = whos;
if ~any(contains({varNames.name},{'BackMean','BackRaw'}))
    load([fileparts(mfilename('fullpath')),'\Data\processedHindlimbAngles.mat'],'BackMean','BackRaw')
end

completeWaves = zeros(size(BackRaw));

completeWaves(:,:,1) = waveFormCompleter(BackRaw(:,:,1),500,500);
figure
completeWaves(:,:,2) = waveFormCompleter(BackRaw(:,:,2),1000,400);
figure
completeWaves(:,:,3) = waveFormCompleter(BackRaw(:,:,3),200,400);

save('.\Data\processedHindlimbAngles.mat','BackMean','BackRaw','completeWaves')