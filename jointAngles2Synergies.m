    close all

    file_dir = fileparts(mfilename('fullpath'));
    load([file_dir,'\Data\processedHindlimbAngles.mat'],'BackMean','BackRaw','completeWaves')
    addpath(genpath(pwd))

    trial = 4;
%     waveform = [BackRaw(:,trial,1)-98,BackRaw(:,trial,2)-90,BackRaw(:,trial,3)-116];
    waveform = [completeWaves(:,trial,1)-98,completeWaves(:,trial,2)-90,completeWaves(:,trial,3)-116];
    
    tstart = tic;
    [obj] = jointMotionInjector(waveform,0);
    telapsed = toc(tstart);
    disp(['Waveforms Injected and Simulated.',' (',num2str(telapsed),'s)'])
    
    % Optimize forces to meet torque demands
    results_cell = obj.pedotti_optimization;
    oforces = results_cell{2,2}';
    if size(oforces,2) ~= 38
        oforces = oforces';
    end
    tstart = tic;
    
%     parfor i = 1:size(oforces,2)
%         if snr(oforces(:,i)) < -2
%             %For those signals that are extremely noisy, perform aggressive smoothing
%             forces(:,i) = smooth(smoothdata(oforces(:,i)),.04);
%         else
%             % For those with low local noise, just apply a low pass filter
%             forces(:,i) = lowpass(oforces(:,i),.08);
%         end
%     end

    % Either smoothing technique works but lowpass distorts the data ends
    %forces = lowpass(oforces,0.01,'Steepness',0.85,'StopbandAttenuation',60);
    forces = smoothdata(oforces,'gaussian',20);
    forces(forces<0) = 0;
    telapsed = toc(tstart);
    disp(['Forces Smoothed.',' (',num2str(telapsed),'s)'])

    tstart = tic;
    synergysort = 'currents';
    switch synergysort
        case 'forces'
            [outk,r2scores,recompiled,W,H] = NMFsyncounter(forces);
            telapsed = toc(tstart);
            if ~isinf(outk)
                disp(['Synergies counted. k = ',num2str(outk),'. (',num2str(telapsed),'s)'])
            else
                disp(['Synergies counted. Over 10 synergies',' (',num2str(telapsed),'s)'])
            end

            % Coefficients of similarity between original forces and recompiled forces from synergies
            coeffMat = pearsonTest(forces,recompiled,0);
    
            % Am signals that will generate the desired forces
            [Am_musc,V_musc] = Am_generator(obj,forces');
            current2inject = 1000.*(V_musc+.06);
            bb = forces;
        case 'currents'
            % Am signals that will generate the desired forces
            [Am_musc,V_musc] = Am_generator(obj,forces');
            current2inject = 1000.*(V_musc+.06);
            [r2scores,recompiled,W,H] = NMFdecomposition(6,current2inject,0);
            bb = current2inject';
    end
    
    if isfile([file_dir,'\Data\h_equations.mat'])
        delete([file_dir,'\Data\h_equations.mat']) 
    end
    
    figure('name','InjectedWaveforms')
    plot(obj.theta_motion)
    title('Joint Angle Waveforms')
    legend({obj.joint_obj{1}.name(4:end),obj.joint_obj{2}.name(4:end),obj.joint_obj{3}.name(4:end)},'Interpreter','none')
    
    figure('name','Current2Inject')
    plot(current2inject')
    title('Inject this current into sim')
    ylabel('Current (nA)')
    
    figure('name','OptimizedVSynergies')
    subplot(2,1,1)
    plot(bb)
    title('Optimized Necessary Forces v. Forces Recompiled from Synergies')
    xlabel('Optimized Muscle Forces')
    ylabel('Forces(N)')
    subplot(2,1,2)
    plot(recompiled)
    xlabel('Forces Recompiled from Synergies')
    ylabel('Forces(N)')
    
    close all
    
    plotWH(current2inject',W,H,0)