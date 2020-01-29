figHandles = get(groot, 'Children');
if ~isempty(figHandles)
    priorFigures = contains({figHandles(:).Name},{'emg'});
    close(figHandles(priorFigures))
end

musc_names = [22,2,3,17,28,16,18,12,11,31,4,7];
st_curve = @(Fmax,steepness,xoff,V) Fmax./(1+exp(steepness*(xoff-V)));
emgMat = reshape(cell2mat(struct2cell(outEMGs)),[100 12]);
emgNorm = .02.*(emgMat./max(emgMat,[],'all'))-.06;
emgNames = lower(fieldnames(outEMGs));
emgNum = length(emgNames);

mname = cell(emgNum,2);
for ii = 1:emgNum
    mname{ii,1} = obj.musc_obj{musc_names(ii)}.muscle_name(4:end);
end
mname(1:emgNum,2) = {'TA2';'IL';'GS';'GA';'VM';'VL';'VI';'RF';'BFP';'SM';'GR';'ST'};

Am_out = zeros(length(emgMat),emgNum);

for ii = 1:emgNum
    muscle = obj.musc_obj{musc_names(ii)};
    EMGcode = mname{ii,2};
    emgMatInd = find(contains(emgNames,lower(mname{ii,2})));
    STmax = muscle.ST_max;
    steepness = muscle.steepness;
    xoff = muscle.x_off;
    waveform = emgNorm(:,emgMatInd);
    Am_out(:,ii) = st_curve(STmax,steepness,xoff,waveform);
end

figure('name','emg')
subplot(2,1,1)
plot(emgNorm*1000)
ylabel('Stimulus (mV)')
subplot(2,1,2)
plot(Am_out)