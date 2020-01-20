sim_datatools = processSimData(sim_path,0);
close all
for ii = 1:length(muscles2check)
    legender{ii} = obj.musc_obj{muscles2check(ii)}.muscle_name;
end

figure
subplot(2,1,1)
plot(current2inject(muscles2check,:)'-60)
legend(legender,'Interpreter','none')
ylim([-60 -45])
subplot(2,1,2)
plot(sim_datatools(3).data(1:end-10,:))
legend(sim_datatools(3).data_cols)

figure
subplot(2,1,1)
plot(forces(:,muscles2check))
legend(legender,'Interpreter','none')
subplot(2,1,2)
plot(sim_datatools(2).data(1:end-10,:))
legend(sim_datatools(2).data_cols)