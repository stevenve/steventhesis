solitary_uniform_10_low_odo = csvread('solitary_uniform_10_low_odo.csv',1,0);
solitary_uniform_100_low_odo = csvread('solitary_uniform_100_low_odo.csv',1,0);
solitary_uniform_1000_low_odo = csvread('solitary_uniform_1000_low_odo.csv',1,0);
solitary_uniform_10000_low_odo = csvread('solitary_uniform_10000_low_odo.csv',1,0);
solitary_uniform_100000_low_odo = csvread('solitary_uniform_100000_low_odo.csv',1,0);
solitary_uniform_1000000_low_odo = csvread('solitary_uniform_1000000_low_odo.csv',1,0);

solitary_uniform_10_low = csvread('solitary_uniform_10_low.csv',1,0);
solitary_uniform_100_low = csvread('solitary_uniform_100_low.csv',1,0);
solitary_uniform_1000_low = csvread('solitary_uniform_1000_low.csv',1,0);
solitary_uniform_10000_low = csvread('solitary_uniform_10000_low.csv',1,0);
solitary_uniform_100000_low = csvread('solitary_uniform_100000_low.csv',1,0);
solitary_uniform_1000000_low = csvread('solitary_uniform_1000000_low.csv',1,0);

solitary_patched_10_low_odo = csvread('solitary_patched_10_low_odo.csv',1,0); 
solitary_patched_100_low_odo = csvread('solitary_patched_100_low_odo.csv',1,0);
solitary_patched_1000_low_odo = csvread('solitary_patched_1000_low_odo.csv',1,0);
solitary_patched_10000_low_odo = csvread('solitary_patched_10000_low_odo.csv',1,0);
solitary_patched_100000_low_odo = csvread('solitary_patched_100000_low_odo.csv',1,0);
solitary_patched_1000000_low_odo = csvread('solitary_patched_1000000_low_odo.csv',1,0);

solitary_patched_10_low = csvread('solitary_patched_10_low.csv',1,0);
solitary_patched_100_low = csvread('solitary_patched_100_low.csv',1,0); 
solitary_patched_1000_low = csvread('solitary_patched_1000_low.csv',1,0); 
solitary_patched_10000_low = csvread('solitary_patched_10000_low.csv',1,0); 
solitary_patched_100000_low = csvread('solitary_patched_100000_low.csv',1,0); 
solitary_patched_1000000_low = csvread('solitary_patched_1000000_low.csv',1,0); 

solitary_uniform_low_odo = solitary_uniform_10_low_odo + solitary_uniform_100_low_odo + solitary_uniform_1000_low_odo + solitary_uniform_10000_low_odo + solitary_uniform_100000_low_odo + solitary_uniform_1000000_low_odo;
solitary_uniform_low = solitary_uniform_10_low + solitary_uniform_100_low + solitary_uniform_1000_low + solitary_uniform_10000_low + solitary_uniform_100000_low + solitary_uniform_1000000_low;
solitary_patched_low_odo = solitary_patched_10_low_odo + solitary_patched_100_low_odo + solitary_patched_1000_low_odo + solitary_patched_10000_low_odo + solitary_patched_100000_low_odo + solitary_patched_1000000_low_odo;
solitary_patched_low = solitary_patched_10_low + solitary_patched_100_low + solitary_patched_1000_low + solitary_patched_10000_low + solitary_patched_100000_low + solitary_patched_1000000_low;

solitary_uniform_low_odo = solitary_uniform_low_odo/6;
solitary_uniform_low = solitary_uniform_low/6;
solitary_patched_low_odo = solitary_patched_low_odo/6;
solitary_patched_low = solitary_patched_low/6;

figure;
hold on;
plot(solitary_uniform_low(:,1),solitary_uniform_low(:,2),'color','blue');
plot(solitary_patched_low(:,1),solitary_patched_low(:,2),'color','green');
plot(solitary_uniform_low_odo(:,1),solitary_uniform_low_odo(:,2),'color','red');
plot(solitary_patched_low_odo(:,1),solitary_patched_low_odo(:,2),'color','black');
hLegend = legend('uniform','patched','uniform w/ odometry','patched w/ odometry');
hTitle = title('Comparison of solitary robot performance with and without odometry in different low density food environments.');
hYlabel = ylabel('Amount of food collected');
hXlabel = xlabel('Time (steps)');
set(gca,'fontsize',16,'fontweight','bold');
set(hTitle, 'FontSize', 16, 'FontWeight' , 'bold');
set(hXlabel, 'FontSize', 16, 'FontWeight' , 'bold');
set(hYlabel, 'FontSize', 16, 'FontWeight' , 'bold');
hold off;


b1 = [solitary_uniform_10_low(100,2), solitary_uniform_100_low(100,2), solitary_uniform_1000_low(100,2), solitary_uniform_10000_low(100,2), solitary_uniform_100000_low(100,2), solitary_uniform_1000000_low(100,2)];
b2 = [solitary_patched_10_low(100,2), solitary_patched_100_low(100,2), solitary_patched_1000_low(100,2), solitary_patched_10000_low(100,2), solitary_patched_100000_low(100,2), solitary_patched_1000000_low(100,2)];
b3 = [solitary_uniform_10_low_odo(100,2), solitary_uniform_100_low_odo(100,2), solitary_uniform_1000_low_odo(100,2), solitary_uniform_10000_low_odo(100,2), solitary_uniform_100000_low_odo(100,2), solitary_uniform_1000000_low_odo(100,2)];
b4 = [solitary_patched_10_low_odo(100,2), solitary_patched_100_low_odo(100,2), solitary_patched_1000_low_odo(100,2), solitary_patched_10000_low_odo(100,2), solitary_patched_100000_low_odo(100,2), solitary_patched_1000000_low_odo(100,2)];
b1=b1';b2=b2';b3=b3';b4=b4';

grouping = [ones(6,1); 2*ones(6,1); 3*ones(6,1); 4*ones(6,1)];
b = [b1;b2;b3;b4];

figure;
hold on;
boxplot(b, grouping,'labels',{'uniform','patched','uniform w/ odometry','patched w/ odometry'});
hTitle = title('Comparison of solitary robot performance with and without odometry in different low density food environments.');
hYlabel = ylabel('Amount of food collected');
set(gca,'fontsize',16,'fontweight','bold');
set(hTitle, 'FontSize', 16, 'FontWeight' , 'bold');
set(hYlabel, 'FontSize', 16, 'FontWeight' , 'bold');
xl = findobj(gca,'Type','text');
set(xl, 'FontSize', 16, 'FontWeight' , 'bold');
hold off;


