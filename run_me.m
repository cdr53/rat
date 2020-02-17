jointAngles2Synergies
% synergyProjectBuilder
indivProjectBuilder

disp('Open the project and create the sim. Press button to continue after.')
pause

sdata = processSimData("G:\My Drive\Rat\SynergyControl\Animatlab\SynergyWalking\SynergyWalking20200109_Standalone.asim",1);

bb = [obj.theta_motion ; sdata(end).data];
figure
subplot(2,1,1)
plot(obj.theta_motion_time,obj.theta_motion,'LineWidth',3)
title('Motor Driven Input Waveforms','FontSize',20)
ylabel('Joint Angle','FontSize',18)
legend({'Hip';'Knee';'Ankle'})
ylim([min(bb,[],'all') max(bb,[],'all')])
subplot(2,1,2)
plot(obj.theta_motion_time,[sdata(end).data(:,3) sdata(end).data(:,2) sdata(end).data(:,1)],'LineWidth',3)
title('Muscle Driven Output Waveforms','FontSize',20)
ylabel('Joint Angle','FontSize',18)
legend({'Hip';'Knee';'Ankle'})
ylim([min(bb,[],'all') max(bb,[],'all')])