clear all

figure()
for i=1:12
    d_real = load(strcat('trial1/data_joint_',int2str(i-1),'.csv'));
    d_sim_1 = load(strcat('sim_1.32_0.22/data_joint_',int2str(i-1),'.csv'));
    d_sim_2 = load(strcat('sim_idfriction_0.22/data_joint_',int2str(i-1),'.csv'));
    d_sim_3 = load(strcat('sim_idfriction_5.22/data_joint_',int2str(i-1),'.csv'));
    d_sim_4 = load(strcat('sim_idfriction_15.22/data_joint_',int2str(i-1),'.csv'));


    subplot(2,6,i)
    plot(d_real(:,i+2))
    hold on
    plot(d_sim_1(:,i+2))
    plot(d_sim_2(:,i+2))
    plot(d_sim_3(:,i+2))
    plot(d_sim_4(:,i+2))
end
