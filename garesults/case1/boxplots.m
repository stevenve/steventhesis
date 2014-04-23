ga = csvread('ga0.csv',1,0);
b1(1) = ga(500,2) + ga(500,3) + ga(500,4);
for n = 1:29
    filename = strcat('ga',int2str(n),'.csv');
    tmp = csvread(filename,1,0);
    b1(n+1) = tmp(500,2) + tmp(500,3) + tmp(500,4);
    ga = ga + tmp;
end
ga(:,5) = ga(:,2) + ga(:,3) + ga(:,4);
ga(:,6) = ga(:,5)./(ga(:,1)/1000);
ga = ga/30;

figure;
hold on;
plot(ga(:,1),ga(:,5),'color','black');
hLegend = legend('patched - best recruitment strat');
hTitle = title('Comparison');
hYlabel = ylabel('Amount of food collected');
hXlabel = xlabel('Time (steps)');
set(gca,'fontsize',16,'fontweight','bold');
set(hTitle, 'FontSize', 16, 'FontWeight' , 'bold');
set(hXlabel, 'FontSize', 16, 'FontWeight' , 'bold');
set(hYlabel, 'FontSize', 16, 'FontWeight' , 'bold');
hold off;


b1 = b1';

grouping = [ones(30,1);];
b = [b1];

figure;
hold on;
boxplot(b, grouping,'labels',{'den eerste'});
hTitle = title('Comparison ');
hYlabel = ylabel('Amount of food collected');
set(gca,'fontsize',16,'fontweight','bold');
set(hTitle, 'FontSize', 16, 'FontWeight' , 'bold');
set(hYlabel, 'FontSize', 16, 'FontWeight' , 'bold');
xl = findobj(gca,'Type','text');
set(xl, 'FontSize', 16, 'FontWeight' , 'bold');
hold off;

figure;
hold on;
plot(ga(:,1),ga(:,6),'color','blue');
legend('1');
hold off;

