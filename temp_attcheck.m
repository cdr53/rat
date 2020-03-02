
varNames = whos;
if isempty(find(contains({varNames.name},'motorObj'),1,'first'))
    motorObj = design_synergy("G:\My Drive\Rat\SynergyControl\Animatlab\SynergyWalking\SynergyWalking20200109_Standalone.asim");
end

muscNum = 35;
attNum = 2;

att2follow = motorObj.musc_obj{muscNum, 1}.pos_attachments{attNum,4};
attName = motorObj.musc_obj{muscNum, 1}.pos_attachments{attNum,2}(3:end-2);

attSim = importdata([pwd,'\Animatlab\SynergyWalking\inscheck.txt']);
oldXYZ = attSim.data;
newXYZ = oldXYZ;
if size(attSim.data,2) == 5
    xyz = 3:5;
    timeInd = 2;
else
    xyz = 2:4;
    timeInd = 1;
end
for ii = xyz
    oldID = attSim.colheaders{ii};
    switch oldID(1)
        case 'X'
            newXYZ(:,xyz(1)) = oldXYZ(:,ii);
        case 'Y'
            newXYZ(:,xyz(2)) = oldXYZ(:,ii);
        case 'Z'
            newXYZ(:,xyz(3)) = oldXYZ(:,ii);
    end
end
attSim.data = newXYZ;

ftemp = figure('Position',[-1919,121,1920,1004]);
subplot(2,1,1)
    plot(motorObj.theta_motion_time,1000.*att2follow,'LineWidth',3)
    xlim([0 10])
    title(attName,'Interpreter','None','FontSize',20)
    ylabel('Expected/Calculated','FontSize',18)
    legend({'X';'Y';'Z'})
subplot(2,1,2)
    plot(attSim.data(:,timeInd),1000.*attSim.data(:,xyz),'LineWidth',3)
    legend({'X';'Y';'Z'})
    xlim([0 10])
    ylabel('Simulation output','FontSize',18)
    
muscAcr = 'GrPIns';
saveas(ftemp,['G:\My Drive\Rat\MeetingFiles\Meeting_20200309\AttCheck_',muscAcr,'.png'])
saveas(ftemp,['G:\My Drive\Rat\MeetingFiles\Meeting_20200309\AttCheck_',muscAcr,'.fig'])