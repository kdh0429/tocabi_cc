clear all

figure()
for i=1:12
    d_real = load(strcat('trial2/data_joint_',int2str(i-1),'.csv'));
    d_sim_1 = load(strcat('sim_1.32_0.22/data_joint_',int2str(i-1),'.csv'));
    d_sim_2 = load(strcat('sim_idfriction_0.22/data_joint_',int2str(i-1),'.csv'));
%     d_sim_3 = load(strcat('sim_idfriction_5.22/data_joint_',int2str(i-1),'.csv'));
%     d_sim_4 = load(strcat('sim_idfriction_15.22/data_joint_',int2str(i-1),'.csv'));
%     d_sim_5 = load(strcat('sim_idfriction_25.22/data_joint_',int2str(i-1),'.csv'));
    d_sim_6 = load(strcat('sim_idfriction_0.22_delay/data_joint_',int2str(i-1),'.csv'));
    d_sim_7 = load(strcat('sim_idfriction_0.22_delay_6/data_joint_',int2str(i-1),'.csv'));


    subplot(2,6,i)
    plot(d_real(:,i+2))
    hold on
%     plot(d_sim_1(:,i+2))
    plot(d_sim_2(:,i+2))
%     plot(d_sim_3(:,i+2))
%     plot(d_sim_4(:,i+2))
%     plot(d_sim_5(:,i+2))
    plot(d_sim_6(:,i+2))
    plot(d_sim_7(:,i+2))
end

figure()
for i=1:12
    d_real = load(strcat('trial2/data_joint_',int2str(i-1),'.csv'));
    d_sim_1 = load(strcat('sim_1.32_0.22/data_joint_',int2str(i-1),'.csv'));
    d_sim_2 = load(strcat('sim_idfriction_0.22/data_joint_',int2str(i-1),'.csv'));
%     d_sim_3 = load(strcat('sim_idfriction_5.22/data_joint_',int2str(i-1),'.csv'));
%     d_sim_4 = load(strcat('sim_idfriction_15.22/data_joint_',int2str(i-1),'.csv'));
%     d_sim_5 = load(strcat('sim_idfriction_25.22/data_joint_',int2str(i-1),'.csv'));


    subplot(2,6,i)
    plot(d_real(:,i+68))
    hold on
    plot(d_sim_1(:,i+68))
    plot(d_sim_2(:,i+68))
%     plot(d_sim_3(:,i+2))
%     plot(d_sim_4(:,i+2))
%     plot(d_sim_5(:,i+2))
end