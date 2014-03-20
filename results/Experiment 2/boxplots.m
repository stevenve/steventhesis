solitary_patched_10 = csvread('solitary_patched_10.csv',1,0);
solitary_patched_100 = csvread('solitary_patched_100.csv',1,0);
solitary_patched_1000 = csvread('solitary_patched_1000.csv',1,0);
solitary_patched_10000 = csvread('solitary_patched_10000.csv',1,0);
solitary_patched_100000 = csvread('solitary_patched_100000.csv',1,0);
solitary_patched_1000000 = csvread('solitary_patched_1000000.csv',1,0);

solitary_uniform_10 = csvread('solitary_uniform_10.csv',1,0);
solitary_uniform_100 = csvread('solitary_uniform_100.csv',1,0);
solitary_uniform_1000 = csvread('solitary_uniform_1000.csv',1,0);
solitary_uniform_10000 = csvread('solitary_uniform_10000.csv',1,0);
solitary_uniform_100000 = csvread('solitary_uniform_100000.csv',1,0);
solitary_uniform_1000000 = csvread('solitary_uniform_1000000.csv',1,0);

combined_patched_10 = csvread('combined_patched_10.csv',1,0); combined_patched_10(:,5) = combined_patched_10(:,2) + combined_patched_10(:,3) + combined_patched_10(:,4);
combined_patched_100 = csvread('combined_patched_100.csv',1,0);combined_patched_100(:,5) = combined_patched_100(:,2) + combined_patched_100(:,3) + combined_patched_100(:,4);
combined_patched_1000 = csvread('combined_patched_1000.csv',1,0);combined_patched_1000(:,5) = combined_patched_1000(:,2) + combined_patched_1000(:,3) + combined_patched_1000(:,4);
combined_patched_10000 = csvread('combined_patched_10000.csv',1,0);combined_patched_10000(:,5) = combined_patched_10000(:,2) + combined_patched_10000(:,3) + combined_patched_10000(:,4);
combined_patched_100000 = csvread('combined_patched_100000.csv',1,0);combined_patched_100000(:,5) = combined_patched_100000(:,2) + combined_patched_100000(:,3) + combined_patched_100000(:,4);
combined_patched_1000000 = csvread('combined_patched_1000000.csv',1,0);combined_patched_1000000(:,5) = combined_patched_1000000(:,2) + combined_patched_1000000(:,3) + combined_patched_1000000(:,4);

combined_uniform_10 = csvread('combined_uniform_10.csv',1,0); combined_uniform_10(:,5) = combined_uniform_10(:,2) + combined_uniform_10(:,3) + combined_uniform_10(:,4);
combined_uniform_100 = csvread('combined_uniform_100.csv',1,0); combined_uniform_100(:,5) = combined_uniform_100(:,2) + combined_uniform_100(:,3) + combined_uniform_100(:,4);
combined_uniform_1000 = csvread('combined_uniform_1000.csv',1,0); combined_uniform_1000(:,5) = combined_uniform_1000(:,2) + combined_uniform_1000(:,3) + combined_uniform_1000(:,4);
combined_uniform_10000 = csvread('combined_uniform_10000.csv',1,0); combined_uniform_10000(:,5) = combined_uniform_10000(:,2) + combined_uniform_10000(:,3) + combined_uniform_10000(:,4);
combined_uniform_100000 = csvread('combined_uniform_100000.csv',1,0); combined_uniform_100000(:,5) = combined_uniform_100000(:,2) + combined_uniform_100000(:,3) + combined_uniform_100000(:,4);
combined_uniform_1000000 = csvread('combined_uniform_1000000.csv',1,0); combined_uniform_1000000(:,5) = combined_uniform_1000000(:,2) + combined_uniform_1000000(:,3) + combined_uniform_1000000(:,4);

solitary_patched = solitary_patched_10 + solitary_patched_100 + solitary_patched_1000 + solitary_patched_10000 + solitary_patched_100000 + solitary_patched_1000000;
solitary_uniform = solitary_uniform_10 + solitary_uniform_100 + solitary_uniform_1000 + solitary_uniform_10000 + solitary_uniform_100000 + solitary_uniform_1000000;
combined_patched = combined_patched_10 + combined_patched_100 + combined_patched_1000 + combined_patched_10000 + combined_patched_100000 + combined_patched_1000000;
combined_uniform = combined_uniform_10 + combined_uniform_100 + combined_uniform_1000 + combined_uniform_10000 + combined_uniform_100000 + combined_uniform_1000000;

solitary_patched = solitary_patched/6;
solitary_uniform = solitary_uniform/6;
combined_patched = combined_patched/6;
combined_uniform = combined_uniform/6;

figure;
hold on;
plot(solitary_uniform(:,1),solitary_uniform(:,2),'color','blue');
plot(combined_uniform(:,1),combined_uniform(:,5),'color','green');
plot(solitary_patched(:,1),solitary_patched(:,2),'color','red');
plot(combined_patched(:,1),combined_patched(:,5),'color','black');
hLegend = legend('solitary uniform','combined uniform','solitary patched','combined patched');
hTitle = title('Comparison of performance of solitary robots vs combined roles in low density uniform and (small) patched environments.');
hYlabel = ylabel('Amount of food collected');
hXlabel = xlabel('Time (steps)');
set(gca,'fontsize',16,'fontweight','bold');
set(hTitle, 'FontSize', 16, 'FontWeight' , 'bold');
set(hXlabel, 'FontSize', 16, 'FontWeight' , 'bold');
set(hYlabel, 'FontSize', 16, 'FontWeight' , 'bold');
hold off;


b1 = [solitary_uniform_10(100,2), solitary_uniform_100(100,2), solitary_uniform_1000(100,2), solitary_uniform_10000(100,2), solitary_uniform_100000(100,2), solitary_uniform_1000000(100,2)];
b2 = [combined_uniform_10(100,5), combined_uniform_100(100,5), combined_uniform_1000(100,5), combined_uniform_10000(100,5), combined_uniform_100000(100,5), combined_uniform_1000000(100,5)];
b3 = [solitary_patched_10(100,2), solitary_patched_100(100,2), solitary_patched_1000(100,2), solitary_patched_10000(100,2), solitary_patched_100000(100,2), solitary_patched_1000000(100,2)];
b4 = [combined_patched_10(100,5), combined_patched_100(100,5), combined_patched_1000(100,5), combined_patched_10000(100,5), combined_patched_100000(100,5), combined_patched_1000000(100,5)];
b1=b1';b2=b2';b3=b3';b4=b4';

grouping = [ones(6,1); 2*ones(6,1); 3*ones(6,1); 4*ones(6,1)];
b = [b1;b2;b3;b4];

figure;
hold on;
boxplot(b, grouping,'labels',{'solitary uniform','combined uniform','solitary patched','combined patched'});
hTitle = title('Comparison of performance of solitary robots vs combined roles in low density uniform and (small) patched environments.');
hYlabel = ylabel('Amount of food collected');
set(gca,'fontsize',16,'fontweight','bold');
set(hTitle, 'FontSize', 16, 'FontWeight' , 'bold');
set(hYlabel, 'FontSize', 16, 'FontWeight' , 'bold');
xl = findobj(gca,'Type','text');
set(xl, 'FontSize', 16, 'FontWeight' , 'bold');
hold off;


