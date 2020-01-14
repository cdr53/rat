function [Fmt,musclelength,musclevelocity,Al,Lr,Lw,dt] = lloyd_work(obj,to_plot)

close all

if nargin < 2
    design_synergy
    to_plot = 0;
end

%% Import Muscle Information (length, object, velocity)
t = obj.theta_motion_time(1:end-9);
dt = obj.dt_motion;
muscle = obj.musc_obj{12};
musclelength = muscle.muscle_length_profile(1:end-9);
musclevelocity = muscle.muscle_velocity_profile(1:end-9);

% ml = muscle.l_min*ones(length(t),1);
% ml(333:582) = linspace(muscle.l_min,muscle.l_max,250);
% ml(583:1183) = muscle.l_max;
% ml(1184:1432) = linspace(muscle.l_max,muscle.l_min,249);
% musclelength = ml;
% musclevelocity = diff(musclelength)./dt;
%     amp = (muscle.l_max-muscle.l_min)/2;
%     offset = (muscle.l_max+muscle.l_min)/2;
%     musclelength = amp*sin(20*t)+offset;
%     musclevelocity = diff(musclelength)./dt;
% Lr = muscle.RestingLength;
Lr = muscle.l_max;
Lw = abs(muscle.l_max-muscle.l_min)/sqrt(.3);

%% Define the stimulus profile
%u = zeros(size(t));
%  u(t>=.5) = 1;
   u = .5*sin(2*t)+.5;

%% Define the force functions (fl,fv,fp, etc)
% Force Length: Hill model
fl = @(Lm,Lr,Lw) 1-(Lm-Lr).^2./Lw^2;

%Activation: Lloyd 2002/Potvin 1996 activation equation 
% a = @(A,u) (exp(A*u)-1)/(exp(A)-1);

%Activation: Thelen 2003 activation equation with activation and deactivation constants
tau_act = .015;
tau_deact = .05;
for i = 1:length(u)
    if i==1
    asim(i) = u(i);
    else
        if u(i-1) > asim(i-1)
            asim(i) = dt*(u(i-1)-asim(i-1))/(tau_act*(.5+.15*asim(i-1)))+asim(i-1);
        else
            asim(i) = dt*((u(i-1)-asim(i-1))*(.5+1.5*asim(i-1))/tau_deact)+asim(i-1);
        end
    end
end

%  fv = @(Vmax,Fm,a,fl,b) (.25+.75.*a).*Vmax.*(Fm-a.*fl)./b;
%  fv = @(Vmax,V) (1-V./Vmax)./(1+4*(V./Vmax));
%% from Brown 1996, eqn (12)
% fv = @(b,a,V) ((b-a.*V)./(V+b));
% a2 = @(L,p,q,r) p.*L.^2+q.*L+r;

%% from Brown 1996, eqn (18)
    % parameters developed from fvsolver_params.m
    % 1.6Fmax at 3Lo/s and 0 at -4Lo/s
fv = @(V) .8*(1+tanh(3.0648*(V+.083337)));

%Thelen 2003, slightly modified eqn (3)
fp = @(k,Lm,eo) (exp(k*(Lm-1)/eo)-1)/(exp(.5*(k/eo))-1);

Lopt = @(Lr,gamma,a) Lr*(gamma*(1-a)+1);

phi = @(Lopt,phiopt,Lm) asin((Lopt.*sin(phiopt))./(Lm));

%% Set parameters for the functions
A = -1;
Vmax = (muscle.vmax_fiber/muscle.lf_lm)/1000;
Fm = 1;
Af = .25;
Flen = 1.5;
k = 6;
eo = .6;
phiopt = muscle.pennation_angle*(pi/180);
gamma = .15;
Fmax = muscle.max_force;
%From Brown 1992 for FV eqn (16)
p = -5.34;
q = 8.41;
r = -4.70;
%From Brown 1992 for FV eqns (10) and (12)
a1 = .17;
b1 = -.69;
b2 = .18;

% just testing!!!
% Lr = (muscle.l_max+muscle.l_min)/2;
% Lw = .602*Lr;

%% Determine the Lloyd force profile
for i=1:length(musclelength)
    Lm = musclelength(i);
    if i == 1
        Lv = musclevelocity(1);
    else
        Lv = musclevelocity(i-1);
    end
    fL(i) = fl(Lm,Lr,Lw);
%   act(i) = a(A,u(i));
    act(i) = asim(i);
%% for use with Brown 1996 eqn (12)   
%     if Lv < 0
%         fV(i) = fv(b1,a1,Lv/Lr);
%     else
%         aa2(i) = a2(Lm/Lr,p,q,r);
%         fV(i) = fv(b2,aa2(i),Lv/Lr);
%    end
%% for use with Brown 1996 eqn (18)
    fV(i) = fv(Lv/Lr);
    Lnorm = Lm/Lr;
    if Lnorm >= 1
        fP(i) = fp(k,Lnorm,eo);
    else
        fP(i) = 0;
    end
    Loptnow(i) = Lopt(Lr,gamma,act(i));
    phinow(i) = phi(Loptnow(i),phiopt,Lm);
    Fmt(i) = Fmax*(fL(i)*fV(i)*act(i)+fP(i))*cos(phinow(i));
    Fmt_noact(i) = Fmax*(fL(i)*fV(i)+fP(i))*cos(phinow(i));
end

% Reverse!
for i = 1:length(Fmt)
    arev(i) = (1/(fL(i)*fV(i)))*(Fmt(i)/(Fmax*cos(phinow(i)))-fP(i));
end

Al = fl(musclelength,Lr,Lw)';
if to_plot
    figure('Position',[500 10 500 1000])
    aplot = subplot(5,1,1);
    plot(obj.theta_motion_time,obj.theta_motion(:,1:3)*(180/pi),'LineWidth',2)
    ylabel('Joint Angle (deg)')
    xlabel('Time (s)')
    title(aplot,'Joint Angles');
    bplot = subplot(5,1,2);
    plot(t,act,'LineWidth',2)
    ylabel('Normalized Activation')
    xlabel('Time (s)')
    title(bplot,'Activation')
    cplot = subplot(5,1,3);
    plot(t(1:end-1),musclevelocity*1000,'LineWidth',2)
    hold on
    zeroline = plot(t(1:end-1),zeros(1,length(musclevelocity)));
    zeroline.Color = [zeroline.Color,.2];
    ylabel('Muscle Velocity (mm/s)')
    xlabel('Time (s)')
    title(cplot,'Muscle Velocity')
    dplot = subplot(5,1,4);
    plot(t,musclelength*1000,'LineWidth',2)
    ylabel('Muscle Length (mm)')
    xlabel('Time (s)')
    title(dplot,'Muscle Length')
    eplot = subplot(5,1,5);
    plot(t,Fmt,'r','LineWidth',2)
    hold on
    tt = plot(t,Fmt_noact,'b','LineWidth',2);
    tt.Color = [tt.Color,.1];
    ylabel('Muscle Force (N)')
    xlabel('Time (s)')
    title(eplot,'Muscle Force')

    figure('Position',[843,32,454,938])
    suba1 = subplot(2,1,1);
    plot(100*(musclelength/Lr),100*fL,'LineWidth',2)
    ylabel('% Maximal Force','FontSize',14)
    xlabel('% Optimal Length','FontSize',14)
    title(suba1,'Force Length Curve','FontSize',18)
    suba2 = subplot(2,1,2);
    plot(musclevelocity/Vmax,100*fV(1:end-1),'LineWidth',2)
    ylabel('% Maximal Force','FontSize',14)
    xlabel('Muscle Velocity (L_o/s)','FontSize',14)
    title(suba2,'Force Velocity Curve','FontSize',18)
end

%% Big fig
if to_plot
    bigfig = figure('Position',[28,21,796,955]);

    subplot(3,3,1)
    plot(t,act,'LineWidth',2)
    title(muscle.muscle_name(4:end))
    ylabel('Activation')

    subplot(3,3,2)
    plot(t,phinow*(180/pi))
    ylabel('Penn Angle')

    subplot(3,3,3)
    plot(t,Loptnow)
    ylabel('Optimal Length')
    
    subplot(3,3,4)
    plot(t(1:end-1),musclevelocity)
    hold on
    plot(t(1:end-1),zeros(length(t(1:end-1)),1))
    ylabel('musclevelocity')

    subplot(3,3,5)
    plot(t,fV)
    ylabel('Force Velocity')

    subplot(3,3,6)
    plot(t,fP)
    ylabel('Force Passive')

    subplot(3,3,7)
    plot(t,musclelength)
    hold on
    plot(t,muscle.l_min*ones(1,length(t)))
    plot(t,muscle.l_max*ones(1,length(t)))
    ylabel('Muscle Length')
    
    subplot(3,3,8)
    plot(t,fL)
    ylabel('Force Length')

    subplot(3,3,9)
    plot(t,Fmt)
    hold on
    fmaxline = ones(1,length(Fmt))*Fmax;
    plot(t,fmaxline)
    ylabel('Force MT')
end
%% FL Curve
if to_plot
    figure('Position',[843,32,454,938])
    suba1 = subplot(3,1,1);
    plot(100*(musclelength/Lr),100*fL,'LineWidth',2)
    ylabel('% Maximal Force')
    xlabel('% Optimal Length')
    title(suba1,'Force Length Curve')

    suba2 = subplot(3,1,2);
    plot(musclevelocity/Vmax,100*fV(1:end-1),'LineWidth',2)
    ylabel('% Maximal Force','FontSize',12)
    xlabel('Muscle Velocity (L_o/s)')
    title(suba2,'Force Velocity Curve')

    suba3 = subplot(3,1,3);
    plot(musclelength/Lr,fP)
    ylabel('Force P')
    xlabel('Muscle Length')
    title(suba3,'Passive Force')
end
%% 3D
if to_plot
    figure('Position',[1333,565,560,420])
    plotvel = musclevelocity/Vmax;
    plotfV = fV(1:end-1);
    plotlen = musclelength(1:end-1)/Lr;
    plot3(plotvel, plotlen,plotfV)
    grid on
    pbaspect([1 1 1])
    view([-52 17])
    xlabel('velocity')
    ylabel('length')
    zlabel('fV')
end
end