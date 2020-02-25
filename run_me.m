jointAngles2Synergies
% synergyProjectBuilder
indivProjectBuilder

musclesim = [pwd,'\Animatlab\SynergyWalking\muscleStim.asim'];

sdata = processSimData(musclesim);

jmInd = find(contains({sdata.name},'JointMotion'));
bb = [obj.theta_motion ; sdata(jmInd).data];
figure
subplot(2,1,1)
    plot(obj.theta_motion_time,obj.theta_motion,'LineWidth',3)
    title('Motor Driven Input Waveforms','FontSize',20)
    ylabel('Joint Angle','FontSize',18)
    legend({'Hip';'Knee';'Ankle'})
    ylim([min(bb,[],'all') max(bb,[],'all')])
        cw = obj.theta_motion(obj.sampling_vector(1):obj.sampling_vector(end),:);
        sw = [sdata(jmInd).data(:,3) sdata(jmInd).data(:,2) sdata(jmInd).data(:,1)];
        cw2 = interp1(1:length(cw),cw,linspace(1,length(cw),length(sw)));
subplot(2,1,2)
    temp1 = plot(sdata(2).time,cw2,'LineWidth',3);
    colorvec = get(temp1,'Color');
    colorvec = [cell2mat(colorvec),[.2 ;.2; .2]];
        for ii = 1:3
            temp1(ii).Color = colorvec(ii,:);
        end
    hold on
    hipInd = find(contains(sdata(jmInd).data_cols,'Hip'),1,'first');
    kneeInd = find(contains(sdata(jmInd).data_cols,'Knee'),1,'first');
    ankleInd = find(contains(sdata(jmInd).data_cols,'Ankle'),1,'first');
    temp2 = plot(sdata(end).time,[sdata(jmInd).data(:,hipInd) sdata(jmInd).data(:,kneeInd) sdata(jmInd).data(:,ankleInd)],'LineWidth',3);
        for ii = 1:3
            temp2(ii).Color = [colorvec(ii,1:3),1];
        end
    legend({'Calculated';'Simulated'})
    title('Muscle Driven Output Waveforms','FontSize',20)
    ylabel('Joint Angle','FontSize',18)
    legend({'Hip';'Knee';'Ankle'})
    ylim([min(bb,[],'all') max(bb,[],'all')])
% 
% obj_fake = design_synergy(sim_file);