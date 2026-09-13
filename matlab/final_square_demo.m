clear; clc;
load("ik_dataset_branchaware_prepared.mat");
load("ik_branchaware_network.mat");
robot=importrobot("../robot/base_fixed.urdf");
robot.DataFormat="struct";
qStart=homeConfiguration(robot);
qGoal=qStart;
qGoal(1).JointPosition=pi/4;
qGoal(2).JointPosition=pi/4;
qGoal(3).JointPosition=pi/4;
qGoal(4).JointPosition=pi/4;
qGoal(5).JointPosition=pi/4;
qGoal(6).JointPosition=pi/4;
[qTraj,~,~]=trapveltraj([[qStart(1:6).JointPosition];[qGoal(1:6).JointPosition]]',20);
figure;
for k=1:size(qTraj,2)
    config=homeConfiguration(robot);
    for j=1:6
        config(j).JointPosition=qTraj(j,k);
    end
    show(robot,config,"PreservePlot",false);
    axis equal; view(3); drawnow;
end