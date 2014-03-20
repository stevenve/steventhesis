solitary_uniform_10_high = csvread('solitary_uniform_10_high.csv',1,0);
solitary_uniform_100_high = csvread('solitary_uniform_100_high.csv',1,0);
solitary_uniform_1000_high = csvread('solitary_uniform_1000_high.csv',1,0);
solitary_uniform_10000_high = csvread('solitary_uniform_10000_high.csv',1,0);
solitary_uniform_100000_high = csvread('solitary_uniform_100000_high.csv',1,0);
solitary_uniform_1000000_high = csvread('solitary_uniform_1000000_high.csv',1,0);

solitary_uniform_10_low = csvread('solitary_uniform_10_low.csv',1,0);
solitary_uniform_100_low = csvread('solitary_uniform_100_low.csv',1,0);
solitary_uniform_1000_low = csvread('solitary_uniform_1000_low.csv',1,0);
solitary_uniform_10000_low = csvread('solitary_uniform_10000_low.csv',1,0);
solitary_uniform_100000_low = csvread('solitary_uniform_100000_low.csv',1,0);
solitary_uniform_1000000_low = csvread('solitary_uniform_1000000_low.csv',1,0);

combined_uniform_10_high = csvread('combined_uniform_10_high.csv',1,0); combined_uniform_10_high(:,5) = combined_uniform_10_high(:,2) + combined_uniform_10_high(:,3) + combined_uniform_10_high(:,4);
combined_uniform_100_high = csvread('combined_uniform_100_high.csv',1,0);combined_uniform_100_high(:,5) = combined_uniform_100_high(:,2) + combined_uniform_100_high(:,3) + combined_uniform_100_high(:,4);
combined_uniform_1000_high = csvread('combined_uniform_1000_high.csv',1,0);combined_uniform_1000_high(:,5) = combined_uniform_1000_high(:,2) + combined_uniform_1000_high(:,3) + combined_uniform_1000_high(:,4);
combined_uniform_10000_high = csvread('combined_uniform_10000_high.csv',1,0);combined_uniform_10000_high(:,5) = combined_uniform_10000_high(:,2) + combined_uniform_10000_high(:,3) + combined_uniform_10000_high(:,4);
combined_uniform_100000_high = csvread('combined_uniform_100000_high.csv',1,0);combined_uniform_100000_high(:,5) = combined_uniform_100000_high(:,2) + combined_uniform_100000_high(:,3) + combined_uniform_100000_high(:,4);
combined_uniform_1000000_high = csvread('combined_uniform_1000000_high.csv',1,0);combined_uniform_1000000_high(:,5) = combined_uniform_1000000_high(:,2) + combined_uniform_1000000_high(:,3) + combined_uniform_1000000_high(:,4);

combined_uniform_10_low = csvread('combined_uniform_10_low.csv',1,0); combined_uniform_10_low(:,5) = combined_uniform_10_low(:,2) + combined_uniform_10_low(:,3) + combined_uniform_10_low(:,4);
combined_uniform_100_low = csvread('combined_uniform_100_low.csv',1,0); combined_uniform_100_low(:,5) = combined_uniform_100_low(:,2) + combined_uniform_100_low(:,3) + combined_uniform_100_low(:,4);
combined_uniform_1000_low = csvread('combined_uniform_1000_low.csv',1,0); combined_uniform_1000_low(:,5) = combined_uniform_1000_low(:,2) + combined_uniform_1000_low(:,3) + combined_uniform_1000_low(:,4);
combined_uniform_10000_low = csvread('combined_uniform_10000_low.csv',1,0); combined_uniform_10000_low(:,5) = combined_uniform_10000_low(:,2) + combined_uniform_10000_low(:,3) + combined_uniform_10000_low(:,4);
combined_uniform_100000_low = csvread('combined_uniform_100000_low.csv',1,0); combined_uniform_100000_low(:,5) = combined_uniform_100000_low(:,2) + combined_uniform_100000_low(:,3) + combined_uniform_100000_low(:,4);
combined_uniform_1000000_low = csvread('combined_uniform_1000000_low.csv',1,0); combined_uniform_1000000_low(:,5) = combined_uniform_1000000_low(:,2) + combined_uniform_1000000_low(:,3) + combined_uniform_1000000_low(:,4);

solitary_uniform_high = solitary_uniform_10_high + solitary_uniform_100_high + solitary_uniform_1000_high + solitary_uniform_10000_high + solitary_uniform_100000_high + solitary_uniform_1000000_high;
solitary_uniform_low = solitary_uniform_10_low + solitary_uniform_100_low + solitary_uniform_1000_low + solitary_uniform_10000_low + solitary_uniform_100000_low + solitary_uniform_1000000_low;
combined_uniform_high = combined_uniform_10_high + combined_uniform_100_high + combined_uniform_1000_high + combined_uniform_10000_high + combined_uniform_100000_high + combined_uniform_1000000_high;
combined_uniform_low = combined_uniform_10_low + combined_uniform_100_low + combined_uniform_1000_low + combined_uniform_10000_low + combined_uniform_100000_low + combined_uniform_1000000_low;

solitary_uniform_high = solitary_uniform_high/6;
solitary_uniform_low = solitary_uniform_low/6;
combined_uniform_high = combined_uniform_high/6;
combined_uniform_low = combined_uniform_low/6;

figure;
hold on;
plot(solitary_uniform_low(:,1),solitary_uniform_low(:,2),'color','blue');
plot(combined_uniform_low(:,1),combined_uniform_low(:,5),'color','green');
plot(solitary_uniform_high(:,1),solitary_uniform_high(:,2),'color','red');
plot(combined_uniform_high(:,1),combined_uniform_high(:,5),'color','black');
hLegend = legend('solitary low','combined low','solitary high','combined high');
hTitle = title('Comparison of performance in environments with a low and high uniform food density.');
hYlabel = ylabel('Amount of food collected');
hXlabel = xlabel('Time (steps)');
set(gca,'fontsize',16,'fontweight','bold');
set(hTitle, 'FontSize', 16, 'FontWeight' , 'bold');
set(hXlabel, 'FontSize', 16, 'FontWeight' , 'bold');
set(hYlabel, 'FontSize', 16, 'FontWeight' , 'bold');
hold off;


b1 = [solitary_uniform_10_low(100,2), solitary_uniform_100_low(100,2), solitary_uniform_1000_low(100,2), solitary_uniform_10000_low(100,2), solitary_uniform_100000_low(100,2), solitary_uniform_1000000_low(100,2)];
b2 = [combined_uniform_10_low(100,5), combined_uniform_100_low(100,5), combined_uniform_1000_low(100,5), combined_uniform_10000_low(100,5), combined_uniform_100000_low(100,5), combined_uniform_1000000_low(100,5)];
b3 = [solitary_uniform_10_high(100,2), solitary_uniform_100_high(100,2), solitary_uniform_1000_high(100,2), solitary_uniform_10000_high(100,2), solitary_uniform_100000_high(100,2), solitary_uniform_1000000_high(100,2)];
b4 = [combined_uniform_10_high(100,5), combined_uniform_100_high(100,5), combined_uniform_1000_high(100,5), combined_uniform_10000_high(100,5), combined_uniform_100000_high(100,5), combined_uniform_1000000_high(100,5)];
b1=b1';b2=b2';b3=b3';b4=b4';

grouping = [ones(6,1); 2*ones(6,1); 3*ones(6,1); 4*ones(6,1)];
b = [b1;b2;b3;b4];

figure;
hold on;
boxplot(b, grouping,'labels',{'solitary low','combined low','solitary high','combined high'});
hTitle = title('Comparison of performance in environments with a low and high uniform food density.');
hYlabel = ylabel('Amount of food collected');
set(gca,'fontsize',16,'fontweight','bold');
set(hTitle, 'FontSize', 16, 'FontWeight' , 'bold');
set(hYlabel, 'FontSize', 16, 'FontWeight' , 'bold');
xl = findobj(gca,'Type','text');
set(xl, 'FontSize', 16, 'FontWeight' , 'bold');
hold off;


