close all

for i=1:38
    muscle = obj.musc_obj{i};
    Fmax(i,1) = muscle.max_force;
    %delL(:,1) = max(muscle.muscle_length_profile-muscle.RestingLength,0);
end
fun = @(x) sum((x./Fmax).^2);
[force_wpass,force_nopass,tau2_wpass,tau2_nopass,moment_output,fval_wpass,fval_nopass,Fmt,telapsed] = obj.optimize_forces(fun,0);

figure
for j = 1:3
    subplot(3,1,j)
    plot(moment_output(1:38,:,j)')
end

figure
subplot(2,1,1)
plot(tau2_wpass)
title('Torque with passive torque')
subplot(2,1,2)
plot(tau2_nopass)
title('Torque with NO passive torque')

figure
plot(Fmt)
title('Muscle Forces')

figure
subplot(2,1,1)
plot(force_wpass')
title('Forces with passive torque')
subplot(2,1,2)
plot(force_nopass')
title('Forces with NO passive torque')

NMFsyncounter(force_nopass)