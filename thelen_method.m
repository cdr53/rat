% Following Thelen 2003 "Generating dynamic simulations of..."
curvars = whos;
if ~any(contains({curvars.name},'obj'))
    [obj,sim_file,joints,bodies,joint_limits,joint_profile,sdata] = design_synergy("G:\My Drive\Rat\SynergyControl\Animatlab\SynergyWalking\SynergyWalking20200109_Standalone.asim");
end
%% Stage 1: Desired Accelerations
startInd = 115;
time = joint_profile(startInd:end-10,1);
q0_exp = joint_profile(startInd:end-10,2:end);

[~,q1_exp] = gradient(q0_exp,obj.dt_motion);
samplingFreq = 1/obj.dt_motion;
[b,a] = butter(4,6/(samplingFreq/2),'low');
q1_exp = filtfilt(b,a,q1_exp);

[~,q2_exp] = gradient(q1_exp,obj.dt_motion);
samplingFreq = 1/obj.dt_motion;
[b,a] = butter(4,6/(samplingFreq/2),'low');
q2_exp = filtfilt(b,a,q2_exp);

kv = 40;
kp = 400;
ku = 10;

%% Stage 2
    [Mout,G,C] = MLS_maker(obj);
    % pattOpts = optimoptions('patternsearch',...
    %         'PlotFcn',{'psplotbestx','psplotbestf','psplotmeshsize'},'UseParallel',true,'FunctionTolerance',1e-4);
              pattOpts = optimoptions('patternsearch','Display','off','UseParallel',true,'FunctionTolerance',1e-4);
    % Prepare a matrix fo muscle information for converting steady state force to steady state activation
    numMuscles = length(obj.musc_obj);
    musc_info = zeros(numMuscles,3);
    for ii = 1:numMuscles
        muscle = obj.musc_obj{ii};
        musc_info(ii,1) = muscle.Kse;
        musc_info(ii,2) = muscle.Kpe;
        musc_info(ii,3) = muscle.RestingLength;
        musc_info(ii,4) = muscle.max_force;
        musc_info(ii,5) = muscle.ST_max;
        musc_info(ii,6) = muscle.steepness;
        musc_info(ii,7) = muscle.x_off;
    end
    % Make moment arm term
    for jointNum=1:3
        moment_output(:,:,jointNum) = compute_joint_moment_arms(obj,jointNum,1);
    end
    momentArmsHip = moment_output(:,:,1);
    momentArmsKnee = moment_output(:,:,2);
    momentArmsAnkle = moment_output(:,:,3);
    
    samplingInds = 1:length(q2_exp);
    samplingInds = floor(linspace(1,length(q2_exp),50));
    
    %% Inertia info
    m1 = obj.body_obj{2}.mass/1000;
    m2 = obj.body_obj{3}.mass/1000;
    m3 = obj.body_obj{4}.mass/1000;

    % Computing the moments of inertia
    MOI = zeros(3,3,3);
    % Femur 
    r1 = 1.7751e-3; L1 = 38.6521e-3; MOI(1,1,1) = (1/12)*m1*L1^2; MOI(2,2,1) = (1/12)*m1*L1^2; MOI(3,3,1) = (1/2)*m1*r1^2;
    % Tibia
    r2 = 1.3745e-3; L2 = 41.46e-3; MOI(1,1,2) = sqrt(((1/12)*m2*L2^2)^2+((1/2)*m2*r2^2)^2); MOI(2,2,2) = sqrt(((1/12)*m2*L2^2)^2+((1/2)*m2*r2^2)^2); MOI(3,3,2) = (1/12)*m2*L2^2;
    % Tibia
    h = 2.846e-3; w = 9.486e-3; d = 20.19e-3; MOI(1,1,3) = (1/12)*m3*(d^2+h^2); MOI(2,2,3) = (1/12)*m3*(d^2+w^2); MOI(3,3,3) = (1/12)*m3*(h^2+w^2);

    gifem = [[eye(3).*[m1;m1;m1] zeros(3,3)];[zeros(3,3) eye(3).*[MOI(1,1,1);MOI(2,2,1);MOI(3,3,1)]]];
    gitib = [[eye(3).*[m2;m2;m2] zeros(3,3)];[zeros(3,3) eye(3).*[MOI(1,1,2);MOI(2,2,2);MOI(3,3,2)]]];
    gifot = [[eye(3).*[m3;m3;m3] zeros(3,3)];[zeros(3,3) eye(3).*[MOI(1,1,3);MOI(2,2,3);MOI(3,3,3)]]];
    
    
    %%
    clear I
    counter = 1; 
for ii = samplingInds
    % Stage 1
    if ii == 1
        q0_feedback = q0_exp(ii,:)';
        q1_feedback = q1_exp(ii,:)';
    end
    q2_d = q2_exp(ii,:)'+kv*(q1_exp(ii,:)'-q1_feedback)+kp*(q0_exp(ii,:)'-q0_feedback);
    
    % Stage 2: Optimization Process
    fun = @(fs) thelen_method_obj_func(fs,musc_info);
    lb = zeros(numMuscles,1);
    ub = musc_info(:,4);
    fs0 = zeros(numMuscles,1);
%     beq = Mout(:,:,ii)*q2_d-G(ii,:)'-C(:,:,ii)*q1_feedback.^2;
    beq(:,counter) = Mout(:,:,ii)*q2_exp(ii,:)'-G(ii,:)'-C(:,:,ii)*q1_exp(ii,:)'.^2;
    Aeq = squeeze(moment_output(1:numMuscles,ii,:))'./1000;
    [fs(:,counter),fVal] = patternsearch(fun,fs0,[],[],Aeq,beq(:,counter),lb,ub,[],pattOpts);
    [outVal,A] = thelen_method_obj_func(fs(:,counter),musc_info);
    
    % Stage 3: Find neuron stimuli to generate activations
    if ii == 1
        A_feedback = A;
    end
    %Ainput = A + ku*(A-A_feedback);
    I(:,counter) = thelen_method_current_solve(A,musc_info);
    disp(num2str(counter))
    counter  = counter +1;  
    %keyboard
    
    % Stage 4
    %q2_feedback = inv(Mout(:,:,ii))*(G(ii,:)'+C(:,:,ii)+Aeq*fs);
end