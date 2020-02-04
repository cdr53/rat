variables = whos;
varNames = {variables(:).name};
scrSz = get(groot, 'ScreenSize');
scW = scrSz(3);
close all

if ~any(contains(varNames,'sdata'))
    sdata = processSimData("G:\My Drive\Rat\SynergyControl\Animatlab\SynergyWalking\SynergyWalking20200109_Standalone.asim",1);
end

syns = sdata(1).data(85:end-10,:);
keymns = sdata(2).data(85:end-10,:);
keymuscs = sdata(3).data(85:end-10,:);
keytens = sdata(4).data(85:end-10,:);
n = length(keytens);
m = 500;

keymns = interp1(1:length(keymns),keymns,linspace(1,length(keymns),m));
keymuscs = interp1(1:length(keymuscs),keymuscs,linspace(1,length(keymuscs),m));
keytens = interp1(1:length(keytens),keytens,linspace(1,length(keytens),m));


musc = 29;
mnInd = find(ismember(sdata(2).data_cols,['neur',num2str(musc)]));
mcInd = find(contains(sdata(3).data_cols,obj.musc_obj{musc}.muscle_name(4:end)));
tnInd = find(contains(sdata(4).data_cols,obj.musc_obj{musc}.muscle_name(4:end)));

xax = linspace(0,100,500);

[ha, pos] = tight_subplot(2, 1, [.03 .03],[.05 .05],[.08 .05]);

% subplot(4,2,1)
% plot(bigH'-60)
% title(obj.musc_obj{musc}.muscle_name(4:end),'Interpreter','none','FontSize',18)
% ylabel('Input Synergy Voltage (mV)')
% subplot(4,2,2)
% plot(syns*1000)
% ylabel('Synergy Neuron Voltage (mV)')
% subplot(2,1,1)
axes(ha(1))
    plot(xax,1000.*V_musc(musc,:)','LineWidth',2)
    title(obj.musc_obj{musc}.muscle_name(4:end),'Interpreter','none','FontSize',18)
    ylabel('MN Activation (mV)')
    hold on
    plot(xax,1000.*keymns(:,mnInd),'LineWidth',2)
    legend({'Calculated';'Simulated'},'Location','northeast')
%subplot(2,1,2)
axes(ha(2))
    plot(xax,forces(:,musc),'LineWidth',2)
    ylabel('Muscle Tension (N)')
    hold on
    plot(xax,keytens(:,tnInd),'LineWidth',2)
    legend({'Calculated';'Simulated'},'Location','northeast')