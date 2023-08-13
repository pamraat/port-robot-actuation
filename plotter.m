%% Test file for lidar_range2xy
% clearvars -except lidarScan
% 
% lidarXY = lidar_range2xy(lidarScan, 0.2, [-120 120]/180*pi);
% 
% ang = linspace(-120/180*pi, 120/180*pi, length(lidarScan));
% 
% figure(1)
% polarplot(ang, lidarScan)
% 
% figure(2)
% hold on
% plot(lidarScan.*cos(ang), lidarScan.*sin(ang));
% plot(lidarXY(1,:), lidarXY(2,:));
% hold off

%% Test file for robot2global
% xyG = robot2global([3 2 pi/2],[0 1]);

%% Test file for global2robot
% xyR = global2robot([3 2 pi/2],[2 2]);

%% Plot file for test cases
% clearvars -except lidarScan
% 
% pose1 = [3, -4, 2*pi/3];
% pose2 = [0, 3, pi/2];
% 
% angRange = [-120, 120]/180*pi;
% robotRad = 0.2;
% lidarXY = lidar_range2xy(lidarScan, robotRad, angRange);
% ang = linspace(angRange(1), angRange(2), length(lidarScan));
% 
% lidarXY = lidarXY';
% ang = ang';
% xyG1 = zeros(length(lidarScan), 2);
% xyG2 = zeros(length(lidarScan), 2);
% 
% for i=1:length(lidarScan)
%     xyG1(i, :) = robot2global(pose1, lidarXY(i, :));
%     xyG2(i, :) = robot2global(pose2, lidarXY(i, :));
% end
% 
% hold on
% % plot(lidarXY(:, 1), lidarXY(:, 2), 'LineWidth', 1);
% plot(xyG1(:,1), xyG1(:,2), 'LineWidth', 1);
% plot(xyG2(:,1), xyG2(:,2), 'LineWidth', 1);
% hold off
% grid on
% legend("Pose 1", "Pose 2", 'Location', 'best')
% title("Lidar Scan Data in Global Frame")
% xlabel("X (meters)");
% ylabel("Y (meters)")
% fontsize(gca,14,"points")
% set(gcf, 'Position',  [400, 150, 600, 500])

%% LimitCmds Input file
% maxV = 0.5;
% wheel2Center = 0.13;
% fwdVel = 0;
% angVel = 0;
% 
% [cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center);

%% Trajectory plot for Backup
% dat = dataStore.truthPose(:, [2 3]);
% plot(dat(:,1), dat(:,2), 'LineWidth', 1)
% grid on
% title("Trajectory of Robot")
% xlabel("X (meters)");
% ylabel("Y (meters)")
% fontsize(gca,14,"points")
% set(gcf, 'Position',  [400, 150, 600, 500])

%% Trajectory plot for different epsilon
% global var1 var2 var3 var4
% % var1 = dataStore.truthPose(:, [2 3]); % epsilon = 0.2
% % var2 = dataStore.truthPose(:, [2 3]); % epsilon = 0.5
% % var3 = dataStore.truthPose(:, [2 3]); % epsilon = 1
% % var4 = dataStore.truthPose(:, [2 3]); % epsilon = 5
% hold on
% plot(var1(:,1), var1(:,2), 'LineWidth', 1);
% plot(var2(:,1), var2(:,2), 'LineWidth', 1);
% plot(var3(:,1), var3(:,2), 'LineWidth', 1);
% plot(var4(:,1), var4(:,2), 'LineWidth', 1);
% hold off
% grid on
% title("Trajectory of Robot for different \epsilon", 'Interpreter', 'tex')
% legend("\epsilon = 0.2", "\epsilon = 0.5", "\epsilon = 1", "\epsilon = 5", 'Location', 'best')
% xlabel("X (meters)");
% ylabel("Y (meters)")
% fontsize(gca,14,"points")
% set(gcf, 'Position',  [400, 150, 600, 500]);

%% Trajectory plot for visitWaypoints
% global var1 var2
% % var1 = dataStore.truthPose(:, [2 3]); % For case 1
% % var2 = dataStore.truthPose(:, [2 3]); % For case 2
% hold on
% plot(var1(:,1), var1(:,2), 'LineWidth', 1)
% plot(var2(:,1), var2(:,2), 'LineWidth', 1)
% grid on
% title("Trajectory of Robot")
% legend("Case 1", "Case 2", 'Location', 'best')
% xlabel("X (meters)");
% ylabel("Y (meters)")
% fontsize(gca,14,"points")
% set(gcf, 'Position',  [400, 150, 600, 500])

%% Trajectory plot for zigzag
% global dataStore
% dat = dataStore.truthPose(:, [2 3]);
% plot(dat(:,1), dat(:,2), 'LineWidth', 1)
% grid on
% title("Trajectory of Robot")
% xlabel("X (meters)");
% ylabel("Y (meters)")
% fontsize(gca,14,"points")
% set(gcf, 'Position',  [400, 150, 600, 500])  