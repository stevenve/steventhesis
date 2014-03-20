combined_patched_10_nest = csvread('combined_patched_10_nest.csv',1,0);combined_patched_10_nest(:,5) = combined_patched_10_nest(:,2) + combined_patched_10_nest(:,3) + combined_patched_10_nest(:,4);
combined_patched_100_nest = csvread('combined_patched_100_nest.csv',1,0);combined_patched_100_nest(:,5) = combined_patched_100_nest(:,2) + combined_patched_100_nest(:,3) + combined_patched_100_nest(:,4);
combined_patched_1000_nest = csvread('combined_patched_1000_nest.csv',1,0);combined_patched_1000_nest(:,5) = combined_patched_1000_nest(:,2) + combined_patched_1000_nest(:,3) + combined_patched_1000_nest(:,4);
combined_patched_10000_nest = csvread('combined_patched_10000_nest.csv',1,0);combined_patched_10000_nest(:,5) = combined_patched_10000_nest(:,2) + combined_patched_10000_nest(:,3) + combined_patched_10000_nest(:,4);
combined_patched_10_nest(:,6) = combined_patched_10_nest(:,5)./(combined_patched_10_nest(:,1)/1000);
combined_patched_100_nest(:,6) = combined_patched_100_nest(:,5)./(combined_patched_100_nest(:,1)/1000);
combined_patched_1000_nest(:,6) = combined_patched_1000_nest(:,5)./(combined_patched_1000_nest(:,1)/1000);
combined_patched_10000_nest(:,6) = combined_patched_10000_nest(:,5)./(combined_patched_10000_nest(:,1)/1000);

combined_patched_10 = csvread('combined_patched_10.csv',1,0); combined_patched_10(:,5) = combined_patched_10(:,2) + combined_patched_10(:,3) + combined_patched_10(:,4);
combined_patched_100 = csvread('combined_patched_100.csv',1,0);combined_patched_100(:,5) = combined_patched_100(:,2) + combined_patched_100(:,3) + combined_patched_100(:,4);
combined_patched_1000 = csvread('combined_patched_1000.csv',1,0);combined_patched_1000(:,5) = combined_patched_1000(:,2) + combined_patched_1000(:,3) + combined_patched_1000(:,4);
combined_patched_10000 = csvread('combined_patched_10000.csv',1,0);combined_patched_10000(:,5) = combined_patched_10000(:,2) + combined_patched_10000(:,3) + combined_patched_10000(:,4);
combined_patched_10(:,6) = combined_patched_10(:,5)./(combined_patched_10(:,1)/1000);
combined_patched_100(:,6) = combined_patched_100(:,5)./(combined_patched_100(:,1)/1000);
combined_patched_1000(:,6) = combined_patched_1000(:,5)./(combined_patched_1000(:,1)/1000);
combined_patched_10000(:,6) = combined_patched_10000(:,5)./(combined_patched_10000(:,1)/1000);


figure;
hold on;
plot(combined_patched_10_nest(:,1),combined_patched_10_nest(:,6),'color','blue');
plot(combined_patched_100_nest(:,1),combined_patched_100_nest(:,6),'color','green');
plot(combined_patched_1000_nest(:,1),combined_patched_1000_nest(:,6),'color','red');
plot(combined_patched_10000_nest(:,1),combined_patched_10000_nest(:,6),'color','black');
legend('nest','nest','nest','nest');
hold off;

figure;
hold on;
plot(combined_patched_10(:,1),combined_patched_10(:,6),'color','blue');
plot(combined_patched_100(:,1),combined_patched_100(:,6),'color','green');
plot(combined_patched_1000(:,1),combined_patched_1000(:,6),'color','red');
plot(combined_patched_10000(:,1),combined_patched_10000(:,6),'color','black');
legend('1','2','3','4');
hold off;


