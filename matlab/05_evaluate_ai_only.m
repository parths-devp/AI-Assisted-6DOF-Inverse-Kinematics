clear; clc;
load("ik_dataset_branchaware_prepared.mat");
load("ik_branchaware_network.mat");
robot=importrobot("../robot/base_fixed.urdf");
robot.DataFormat="struct";
YPredNorm=predict(net,XTestNorm);
YPred=double(YPredNorm.*YStd+YMean);
YTest=double(YTest);
jointError=atan2(sin(YPred-YTest),cos(YPred-YTest));
meanJointError=rad2deg(mean(abs(jointError),1));
maxJointError=rad2deg(max(abs(jointError),[],1));
NTest=size(XTest,1);
positionError=zeros(NTest,1);
for i=1:NTest
    config=homeConfiguration(robot);
    for j=1:6
        config(j).JointPosition=YPred(i,j);
    end
    T=getTransform(robot,config,"jaw6_1",robot.BaseName);
    positionError(i)=norm(tform2trvec(T)-XTest(i,1:3));
end
fprintf("Mean joint error: %.2f deg\n",mean(meanJointError));
fprintf("Max joint error: %.2f deg\n",max(maxJointError));
fprintf("Mean position error: %.2f mm\n",mean(positionError)*1000);
fprintf("Median position error: %.2f mm\n",median(positionError)*1000);
fprintf("Max position error: %.2f mm\n",max(positionError)*1000);
fprintf("Within 5 cm: %.2f %%\n",mean(positionError<=0.05)*100);
fprintf("Within 10 cm: %.2f %%\n",mean(positionError<=0.10)*100);
fprintf("Within 15 cm: %.2f %%\n",mean(positionError<=0.15)*100);
save("ai_only_results.mat","positionError","meanJointError","maxJointError");