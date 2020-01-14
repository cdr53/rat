close all

vars = struct2cell(whos);

if ~any(contains(vars(1,:),'obj'))
    design_synergy
end

[act_wpass,act_nopass,tau2_wpass,tau2_nopass,moment_output,fval_wpass,fval_nopass,Fmt] = obj.compute_forces_for_torque(2,0);

figure
subplot(2,3,1)
plot(act_wpass')
ylabel('Forces INCLUDING passive torque')
subplot(2,3,2)
plot(tau2_wpass)
ylabel('Torque INCLUDING passive torque')
subplot(2,3,3)
plot(fval_wpass)
ylabel('Fval INCLUDING passive torque')
subplot(2,3,4)
plot(act_nopass')
ylabel('Forces without passive torque')
subplot(2,3,5)
plot(tau2_nopass)
ylabel('Torque without passive torque')
subplot(2,3,6)
plot(fval_nopass)
ylabel('Fval without passive torque')