function lloyd_parameters(obj)
close all

if nargin < 1
    design_synergy
end

[Fmt,musclelength,musclevelocity,Al,Lr,Lw,dt] = lloyd_work(obj,0);

%% Try to find Ks, Kp, B, and Am values that satisfy the force and length functions
fl = @(Lm,Lr,Lw) 1-(Lm-Lr).^2./Lw^2;

[~,peakinds] = findpeaks(-Fmt);
Tdot_lloyd = gradient(Fmt,dt);

for i = 1:5
    sampleinds = randi(length(Fmt),1,1);

    T = Fmt(sampleinds);
    Tdot_par = Tdot_lloyd(sampleinds);
    L = musclelength(sampleinds)';
    Ldot = musclevelocity(sampleinds)';
    Al_par = Al(sampleinds);

    lloydopt = @(x) lloyd2animatlab(x,Tdot_par(1),Al_par(1),T(1),Ldot(1),L(1),Lr);

   % [a,b] = zipper(Fmt,Tdot_lloyd,musclelength,musclevelocity,Al);
    
    
    % Find possible muscle parameter solutions
    rng default % for reproducibility
    N = 100; % try 100 random start points
    pts = 1000*rand(N,4);
    soln = zeros(N,4); % allocate solution
    fval = zeros(N,1); % allocate solution
    opts = optimoptions('fsolve','Algorithm','levenberg-marquardt','Display','off','MaxFunctionEvaluations',1000);
    for k = 1:N
        [soln(k,:),fval(k,1)] = fsolve(lloydopt,pts(k,:),opts); % find solutions
    end
    count = 1;
    xopts = [];
    for jj = 1:length(soln)
        if sum(soln(jj,:) > 0) == 4
            xopts(count,:) = soln(jj,:);
            count = count+1;
        end
    end
    if ~isempty(xopts)
        break
    end
end

x = xopts(11,:);
Feqn = @(x) (x(1)./x(2)).*(x(3).*max(0,musclelength-Lr)+x(2).*[0;musclevelocity]-(1+x(3)./x(1)).*Fmt'+x(4).*Al');
Tdot_sim = Feqn(x);

figure
subplot(5,1,1)
plot(Tdot_lloyd)
hold on
plot(Tdot_sim)
subplot(5,1,2)
plot((x(1)./x(2)).*x(3).*max(0,musclelength-Lr))
subplot(5,1,3)
plot((x(1)./x(2)).*x(2).*[0;musclevelocity])
subplot(5,1,4)
plot((x(1)./x(2)).*(-(1+x(3)./x(1)).*Fmt'))
subplot(5,1,5)
plot((x(1)./x(2)).*x(4).*Al')


x = xopts(1,:);
ks = x(1);
b = x(2);
kp = x(3);
Am = x(4);

a = dt*ks/b;
c = (1-a*(1+(kp/ks)));
Tdot = @(ks,b,kp,Am) (ks./b).*(kp.*max(0,musclelength(1:end-1)-Lr)+b.*musclevelocity-(1+kp./ks).*Fmt(1:end-1)'+Am.*fl(musclelength(1:end-1),Lr,Lw));
Tdotsim = Tdot(x(1),x(2),x(3),x(4));
Tdot_lloyd = (diff(Fmt))./dt;
tdotsimplot = subplot(2,1,1);
plot(Tdotsim)
title(tdotsimplot,'Simulated Output')
lloydfmt = subplot(2,1,2);
plot(Tdot_lloyd)
title(lloydfmt,'Lloyd eqn Tdot')

    function [soln,fval] = zipper(Fmt,Tdot_lloyd,musclelength,musclevelocity,Al)
        soln = zeros(length(Fmt),4);
        fval = zeros(length(Fmt),1);
        for ii=1:length(Fmt)-1
            Tz = Fmt(ii);
            Tdot_parz = Tdot_lloyd(ii);
            Lz = musclelength(ii)';
            Ldotz = musclevelocity(ii)';
            Al_parz = Al(ii);

            lloydoptz = @(x) lloyd2animatlab(x,Tdot_parz,Al_parz,Tz,Ldotz,Lz,Lr);
            
            optsz = optimoptions('fsolve','Algorithm','levenberg-marquardt','Display','off','MaxFunctionEvaluations',1000);
            [soln(ii,:),fval(ii,1)] = fsolve(lloydoptz,randi(1000,1,4),optsz);
        end
        
        counter = 1;
        possolns = [];
        for jj = 1:length(soln)
            if sum(soln(jj,:) > 0) == 4
                possolns(counter,:) = soln(jj,:);
                counter = counter+1;
            end
        end
        
        Tdot2 = @(ks,b,kp,Am) (ks./b).*(kp.*max(0,musclelength(1:end-1)-Lr)+b.*musclevelocity-(1+kp./ks).*Fmt(1:end-1)'+Am.*fl(musclelength(1:end-1),Lr,Lw));
        for ii = 1:length(possolns)
            x2 = possolns(ii,:);
            Tdotsim2 = Tdot2(x2(1),x2(2),x2(3),x2(4));
            diffvec(ii,1) = sum(abs(Tdotsim2)-abs(Tdot_lloyd(1:end-1)'));
        end
        [~,bb] = min(abs(diffvec));
        possolns(bb,:)
    end

end