%% Final AI-Assisted IK Demonstration
% 6-DOF robot: Cartesian square trajectory
% The neural network provides an initial joint configuration and MATLAB
% numerical IK performs the final pose refinement.

clear; clc; close all;

load("ik_dataset_branchaware_prepared.mat");
load("ik_branchaware_network.mat");

robot = importrobot("../robot/base_fixed.urdf");
robot.DataFormat = "struct";

ik = inverseKinematics("RigidBodyTree", robot);
weights = [1 1 1 1 1 1];

Nside = 40;

p1 = [-0.50 -0.10 0.60];
p2 = [-0.40 -0.10 0.60];
p3 = [-0.40  0.00 0.60];
p4 = [-0.50  0.00 0.60];

side1 = [linspace(p1(1),p2(1),Nside)', ...
         linspace(p1(2),p2(2),Nside)', ...
         linspace(p1(3),p2(3),Nside)'];
side2 = [linspace(p2(1),p3(1),Nside)', ...
         linspace(p2(2),p3(2),Nside)', ...
         linspace(p2(3),p3(3),Nside)'];
side3 = [linspace(p3(1),p4(1),Nside)', ...
         linspace(p3(2),p4(2),Nside)', ...
         linspace(p3(3),p4(3),Nside)'];
side4 = [linspace(p4(1),p1(1),Nside)', ...
         linspace(p4(2),p1(2),Nside)', ...
         linspace(p4(3),p1(3),Nside)'];

trajectory = [side1(1:end-1,:); side2(1:end-1,:); ...
              side3(1:end-1,:); side4];

N = size(trajectory,1);
orientation = eul2tform([pi 0.9599 0]);

currentConfig = homeConfiguration(robot);
previousQ = [currentConfig(1:6).JointPosition];

jointTrajectory = zeros(N,6);
actualPosition = zeros(N,3);
positionError = zeros(N,1);

figure("Name","AI-Assisted IK - Square Trajectory");
show(robot,currentConfig,"PreservePlot",false);
hold on;
plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3),"k--","LineWidth",1.5);
grid on; axis equal; view(3);
xlabel("X (m)"); ylabel("Y (m)"); zlabel("Z (m)");
title("AI-Assisted Inverse Kinematics");

fprintf("\nFINAL AI-IK DEMONSTRATION\n");
fprintf("Square size: 10 cm x 10 cm\n");
fprintf("Trajectory points: %d\n",N);

for i = 1:N
    targetPosition = trajectory(i,:);
    targetPose = trvec2tform(targetPosition) * orientation;

    input = [targetPosition, tform2quat(orientation), previousQ];
    inputNorm = (input-XMean)./XStd;
    predictionNorm = predict(net,inputNorm);
    prediction = double(predictionNorm.*YStd+YMean);

    prediction = max(prediction,-pi+1e-5);
    prediction = min(prediction, pi-1e-5);

    aiConfig = homeConfiguration(robot);
    for j = 1:6
        aiConfig(j).JointPosition = prediction(j);
    end

    [solution,~] = ik("jaw6_1",targetPose,weights,aiConfig);
    jointTrajectory(i,:) = [solution(1:6).JointPosition];

    T = getTransform(robot,solution,"jaw6_1",robot.BaseName);
    actualPosition(i,:) = tform2trvec(T);
    positionError(i) = norm(actualPosition(i,:)-targetPosition);

    previousQ = jointTrajectory(i,:);

    show(robot,solution,"PreservePlot",false);
    hold on;
    plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3),"k--","LineWidth",1.5);
    plot3(actualPosition(1:i,1),actualPosition(1:i,2),actualPosition(1:i,3),"b-","LineWidth",2);
    plot3(targetPosition(1),targetPosition(2),targetPosition(3),"ro",...
          "MarkerSize",7,"MarkerFaceColor","r");
    grid on; axis equal; view(3);
    xlabel("X (m)"); ylabel("Y (m)"); zlabel("Z (m)");
    title(sprintf("AI-Assisted IK | Point %d/%d | Error = %.4f mm",...
          i,N,positionError(i)*1000));
    drawnow;
    pause(0.02);
end

fprintf("\nFINAL RESULTS\n");
fprintf("Mean position error   = %.4f mm\n",mean(positionError)*1000);
fprintf("Median position error = %.4f mm\n",median(positionError)*1000);
fprintf("Maximum position error = %.4f mm\n",max(positionError)*1000);
fprintf("Within 5 cm  = %.2f %%\n",mean(positionError <= 0.05)*100);
fprintf("Within 10 cm = %.2f %%\n",mean(positionError <= 0.10)*100);
fprintf("Within 15 cm = %.2f %%\n",mean(positionError <= 0.15)*100);

fprintf("\nJOINT ANGLE RANGES\n");
for j = 1:6
    fprintf("J%d: %.2f° to %.2f°\n",j,...
        rad2deg(min(jointTrajectory(:,j))),...
        rad2deg(max(jointTrajectory(:,j))));
end

save("final_ai_ik_square_demo.mat",...
     "trajectory","actualPosition","jointTrajectory","positionError");
