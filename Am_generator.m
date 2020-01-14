function [Am_musc,V_musc] = Am_generator(obj,forces)
    if size(forces,2)<size(forces,1)
        forces = forces';
    end
    [beg,ennd,~] = find_step_indices(obj);
    odt = obj.dt_motion;
    dt = ((ennd-beg)*odt)/length(forces);
    forces_dot = gradient(forces,dt);

    fl = @(Lm,Lr,Lw) 1-(Lm-Lr).^2./Lw^2;
    Am = @(Al,b,ks,T_dot,kp,delL,L_dot,T) (1./Al).*((b./ks).*T_dot-kp.*delL-b.*L_dot+(1+kp./ks).*T);
    V = @(A1,A2,A3,A) A1-(1./A3).*log((A2-A)./A);
    st_curve = @(Fmax,steepness,xoff,V) Fmax./(1+exp(steepness*(xoff-V)));
    Am_musc = zeros(size(forces));
    Al_musc_all = Am_musc;
    V_musc = Am_musc;

    for ii = 1:38
       [b,ks,kp,Lw,Lr,xoff,Fmax,steepness,mL,mV] = getMuscParams(obj,ii,beg,ennd);
       delL_musc = max(mL-Lr,0);
       Al_musc = fl(mL,Lr,Lw);
       Tdot = forces_dot(ii,:);
       T = forces(ii,:);
       Am_musc(ii,:) = Am(Al_musc',b,ks,Tdot,kp,delL_musc',mV',T);
       V_musc(ii,:) = real(V(xoff,Fmax,steepness,max(Am_musc(ii,:),0)));
       V_musc(ii,isinf(V_musc(ii,:)))=-.06;
       Al_musc_all(ii,:) = 1./Al_musc;
       %% For loop plotter 1: Plot 5 subplot fig of tension equation
       if 0
       figure
        subplot(5,1,1)
            plot((1+kp./ks).*T)
            xlabel('(1+kp./ks).*T')
        subplot(5,1,2)
            plot((b./ks).*Tdot)
            xlabel('(b./ks).*Tdot')
        subplot(5,1,3)
            plot(kp.*delL_musc)
            xlabel('kp.*delL_musc')
        subplot(5,1,4)
            plot(b.*mV)
            xlabel('b.*mV')
        subplot(5,1,5)
            plot(Am_musc(ii,:),'LineWidth',2)
            hold on
            plot(zeros(length(Am_musc(ii,:)),1))
            %ylim([0 max(Am_musc(ii,:))*1.1])
       end
       %% For loop plotter 2: View one muscle's active and passive waveforms. When active is > passive, Am is possible.
       if 0
            active = (b/ks).*Tdot+(1+kp/ks).*T;
            passive = kp.*delL_musc+b.*mV;
            yLims = [min([active',passive],[],'all') max([active',passive],[],'all')];
            figure
            plot(active,'r','LineWidth',2)
            hold on
            plot(passive,'b','LineWidth',2)
            legend({'Active','Passive'})
            ylim(yLims)
       end
       %%
    end

    outT = inputAmoutputT(obj,Am_musc,Al_musc_all,dt,beg,ennd);

    if 0
        figure
        plot(outT')
        title('Setting Am < 0 to zero')
        figure
        plot(forces')
        title('Original forces trying to recreate')
    end
    %% inputAmoutputT
    function outT = inputAmoutputT(obj,Am,Al,dt,beg,ennd)
        if any(any(Am<0))
            Am(Am<0) = 0;
        end
        for j = 1:38
            [bhold,kshold,kphold,~,Lrhold,~,~,~,mLhold,mVhold] = getMuscParams(obj,j,beg,ennd);
            a = dt*kshold/bhold;
            c = (1-a*(1+(kphold/kshold)));

            for i = 1:length(mVhold)
                if i == 1
    %                 T(j,i) = ks*max(0,mL(i)-Lr)+a*b*mV(i)+a*Am(j,i)*Al(j,i);
                    Thold(j,i) = 0;
                else
                    Thold(j,i) = c*Thold(j,i-1) + (a*kphold)*max(0,mLhold(i-1)-Lrhold) + (a*bhold)*mVhold(i-1)+a*Am(j,i-1)*Al(j,i-1);
                end
            end
        end
        outT = Thold;
    end
    %% getMuscParams
    function [b,ks,kp,Lw,Lr,xoff,Fmax,steepness,mL,mV] = getMuscParams(obj,mnum,beg,ennd)
        n = 500;
        m = length(obj.theta_motion(beg:ennd));
       musc = obj.musc_obj{mnum};
       b = musc.damping;
       ks = musc.Kse;
       kp = musc.Kpe;
       if mnum == 8
        Lw = musc.l_width*1.5;
       else
        Lw = musc.l_width;
       end
       Lr = musc.RestingLength;
      xoff = musc.x_off;
       Fmax = musc.max_force;
       steepness = musc.steepness;
       mL = interp1(1:m,musc.muscle_length_profile(beg:ennd),linspace(1,m,n))';
       mV = interp1(1:m,musc.muscle_velocity_profile(beg:ennd),linspace(1,m,n))';
    end
    %% findParamsfromLloyd
    function x = findParamsFromLloyd(Tdot,Al_musc,T,mV,mL,Lr)
        % Try to find ks, b, kp, Am values that satisfy an input force waveform
        % Doesn't really work atm
        lloydopt = @(x) lloyd2animatlab(x,Tdot',Al_musc,T',mV,mL,Lr);

        % Find possible muscle parameter solutions
        rng default % for reproducibility
        N = 100; % try 100 random start points
        pts = 1000*rand(N,4);
        soln = zeros(N,4); % allocate solution
        fval = zeros(N,1); % allocate solution
        opts = optimoptions('fsolve','Algorithm','levenberg-marquardt','Display','off','MaxFunctionEvaluations',10e6,'PlotFcn',@optimplotfirstorderopt,...
            'FunctionTolerance',1);

        lb = [0,0,0,0];
        ub = [10e4,10e4,10e4,10e4];
        A = [];
        B = [];
        Aeq = [];
        beq = [];
        x0 = [1000,10,100,10];
        mL_rel = max(mL-Lr,0);
        lloydopt = @(x) lloyd2animatlab(x,Tdot(1),Al_musc(1),T(1),mV(1),mL(1),Lr);
        fun = @(x) Tdot(1)-(x(1)./x(2)).*(x(3).*mL_rel(1)+x(2).*mV(1)-(1+x(3)./x(1)).*T(1)+x(4).*Al_musc(1));
        [x,fval,exitflag,output] = fmincon(lloydopt,x0,A,B,Aeq,beq,lb,ub);

        for k = 1:N
            [soln(k,:),fval(k,1)] = fsolve(lloydopt,pts(k,:),opts); % find solutions
        end
    end
end
