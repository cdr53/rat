jointAngles2Synergies
% synergyProjectBuilder
indivProjectBuilder

disp('Open the project and create the sim. Press button to continue after.')
pause

sim_file = "G:\My Drive\Rat\SynergyControl\Animatlab\SynergyWalking\SynergyWalking20200109_Standalone.asim";
sdata = processSimData(sim_file,1);

bb = [obj.theta_motion ; sdata(end).data];
figure
subplot(2,1,1)
plot(obj.theta_motion_time,obj.theta_motion,'LineWidth',3)
title('Motor Driven Input Waveforms','FontSize',20)
ylabel('Joint Angle','FontSize',18)
legend({'Hip';'Knee';'Ankle'})
ylim([min(bb,[],'all') max(bb,[],'all')])
    cw = obj.theta_motion(obj.sampling_vector(1):obj.sampling_vector(end),:);
    sw = [sdata(end).data(:,3) sdata(end).data(:,2) sdata(end).data(:,1)];
    cw2 = interp1(1:length(cw),cw,linspace(1,length(cw),length(sw)));
subplot(2,1,2)
temp1 = plot(sdata(end).time,cw2,'LineWidth',3);
colorvec = get(temp1,'Color');
colorvec = [cell2mat(colorvec),[.2 ;.2; .2]];
    for ii = 1:3
        temp1(ii).Color = colorvec(ii,:);
    end
hold on
temp2 = plot(sdata(end).time,[sdata(end).data(:,3) sdata(end).data(:,2) sdata(end).data(:,1)],'LineWidth',3);
    for ii = 1:3
        temp2(ii).Color = [colorvec(ii,1:3),1];
    end
legend({'Calculated';'Simulated'})
title('Muscle Driven Output Waveforms','FontSize',20)
ylabel('Joint Angle','FontSize',18)
legend({'Hip';'Knee';'Ankle'})
ylim([min(bb,[],'all') max(bb,[],'all')])

obj_fake = design_synergy(sim_file);