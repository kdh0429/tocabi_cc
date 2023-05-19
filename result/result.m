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
