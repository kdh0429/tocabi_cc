d1 = load('data.csv');
d2 = load('data_low_frictionloss.csv');

figure();
for i=1:12
    subplot(2,6,i);
    plot(d1(:,i));
    hold on
    plot(d2(:,i));
end

figure();
for i=13:33
    subplot(4,6,i-12);
    plot(d1(:,i));
    hold on
    plot(d2(:,i));
end

%%
clear all
d3 = load('data.csv');

figure();

plot(d3(:,1),d3(:,6))
hold on
plot(d3(:,1),d3(:,12))

%% Value Function
% 1                writeFile << (rd_cc_.control_time_us_ - start_time_)/1e6 << "\t";
% 2                writeFile << phase_ << "\t";
% 3                writeFile << DyrosMath::minmax_cut(rl_action_(num_action-1)*1/250.0, 0.0, 1/250.0) << "\t";
% 
% 4-9                  writeFile << rd_cc_.LF_FT.transpose() << "\t";
% 10-15                writeFile << rd_cc_.RF_FT.transpose() << "\t";
% 16-21                writeFile << rd_cc_.LF_CF_FT.transpose() << "\t";
% 22-27                writeFile << rd_cc_.RF_CF_FT.transpose() << "\t";
% 
% 28-60                writeFile << rd_cc_.torque_desired.transpose()  << "\t";
% 61-93                writeFile << q_noise_.transpose() << "\t";
% 94-126                writeFile << q_dot_lpf_.transpose() << "\t";
% 127-165                writeFile << rd_cc_.q_dot_virtual_.transpose() << "\t";
% 166-205                writeFile << rd_cc_.q_virtual_.transpose() << "\t";

% 206 207                writeFile << value_ << "\t" << stop_by_value_thres_;
clear d
d = load('data.csv');

figure()
yyaxis left
plot(d(:,1),d(:,206), 'LineWidth', 7)
ylabel('Value','FontSize', 40, 'FontWeight','bold')
yyaxis right
plot(d(:,1),d(:,end), 'LineWidth', 7)
ylabel('IsStopped','FontSize', 40, 'FontWeight','bold')

set(gca,'FontSize',20, 'FontWeight','bold')
title('Emergent Stop Using Value Function','FontSize', 50)
xlabel('Time(s)','FontSize', 14, 'FontWeight','bold')
legend('Value','Stopped','FontSize', 50, 'FontWeight','bold')
grid on
ax = gca;

ax.GridColor = [0 0 0];
ax.GridLineStyle = '-';
ax.GridAlpha = 0.5;

figure()
plot(d(:,167))

figure()
plot(d(:,[6,12]),'DisplayName','d(:,[6,12])')
%%
figure()
for i=1:12
    subplot(2,6,i)
    plot(d(1:1000,1),d(1:1000,132+i))
    hold on
    plot(d2(1:1000,1),d2(1:1000,132+i))
end

figure()
plot(d(1:1000,1),d(1:1000,6))
hold on
plot(d(1:1000,1),d(1:1000,12))
plot(d2(1:1000,1),d2(1:1000,6))
plot(d2(1:1000,1),d2(1:1000,12))

%%
del_tau = d(2:end,28:39) - d(1:end-1,28:39);
mean(abs(del_tau))
