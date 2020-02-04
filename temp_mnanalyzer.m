variables = whos;
varNames = {variables(:).name};
scrSz = get(groot, 'ScreenSize');
scW = scrSz(3);

if ~any(contains(varNames,'sdata'))
    sdata = processSimData("G:\My Drive\Rat\SynergyControl\Animatlab\SynergyWalking\SynergyWalking20200109_Standalone.asim",1);
end

muscnum = 2;
dstart = 85;
mndataInd = find(ismember(sdata(2).data_cols,['neur',num2str(muscnum)]));
mnsim_results = sdata(2).data(dstart:end-10,mndataInd);
muscinfoInd = find(contains(sdata(3).data_cols,obj.musc_obj{muscnum}.muscle_name(4:end)));
muscinfo = sdata(3).data(dstart:end-10,muscinfoInd);
input_res = V_musc(muscnum,:)';

yMax = max([max(mnsim_results) max(muscinfo(10:end-10))]);
yMin = min([min(mnsim_results) min(muscinfo(10:end-10))]);

figHandles = get(groot, 'Children');
if ~isempty(figHandles)
    priorFigures = contains({figHandles(:).Name},{obj.musc_obj{muscnum}.muscle_name(4:end),'inwaves'});
    close(figHandles(priorFigures))
end

figure('Position',[-(scW-10) 130 scW/2 985],'name',obj.musc_obj{muscnum}.muscle_name(4:end))
subplot(4,1,1)
    plot(sdata(2).time(dstart:end-10),mnsim_results)
    ylim([yMin yMax])
    title(['MN Simulation results; ',['neur',num2str(muscnum)]])
subplot(4,1,2)
    plot(sdata(3).time(dstart:end-10),muscinfo)
    ylim([yMin yMax])
    title('Muscle membrane voltage simulation results')
subplot(4,1,3)
    muscle = obj.musc_obj{muscnum};
    STmax = muscle.ST_max;
    xoff = muscle.x_off;
    steep = muscle.steepness;
    st_curve = @(Fmax,steepness,xoff,V) Fmax./(1+exp(steepness*(xoff-V)));
    sim_tension = st_curve(STmax,steep,xoff,muscinfo(10:end-10));
    plot(sim_tension)
    title('Am activation for the above muscle membrane voltage')
subplot(4,1,4)
    plot(Am_musc(muscnum,:)')
    title('Desired Am')


numinlinks = length(nsys.neuron_objects(muscnum).inlinks);
neuron = nsys.neuron_objects(muscnum);
totdata = zeros(500,1);

figure('Position',[-(scW-10)/2 130 scW/2 985],'name','inwaves')
for ii = 1:numinlinks
    neurStr = neuron.inlinks(ii).origin_ID;
    synNum = str2double(neurStr(regexp(neurStr,'\d')))-38;
    ndataind = find(contains(sdata(1).data_cols,neurStr(1:end-3)));
    nsimdata = sdata(1).data(dstart:end-10,ndataind);
    mndata = relW(muscnum,synNum)*bigH(synNum,:)';
        %nsimdata = 1000.*(interp1(1:length(nsimdata),nsimdata,linspace(1,length(nsimdata),length(mndata)))'+.06);
        nsimdata = relW(muscnum,synNum)*interp1(1:length(nsimdata),nsimdata,linspace(1,length(nsimdata),length(mndata)))';
    totdata = totdata+mndata;
    subplot(numinlinks+1,1,ii)
    plot(mndata)
    hold on
    plot(nsimdata)
    legend({'Calculated Wave';'Simulation Results'})
    title([neurStr,'; Synergy ',num2str(synNum)])
end
subplot(numinlinks+1,1,ii+1)
plot(totdata)
hold on
plot(interp1(1:length(mnsim_results),mnsim_results,linspace(1,length(mnsim_results),length(totdata))))
%ylim([yMin yMax])
legend({'Calculated Wave';'Simulation Results'})
title('Total Calculated Input Voltage')