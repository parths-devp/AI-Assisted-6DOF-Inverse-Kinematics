clear; clc;
load("ik_dataset_branchaware_prepared.mat");
load("ik_branchaware_network.mat");
robot=importrobot("../robot/base_fixed.urdf");
robot.DataFormat="struct";
ik=inverseKinematics("RigidBodyTree",robot);
weights=[1 1 1 1 1 1];
YPredNorm=predict(net,XTestNorm);
YPred=double(YPredNorm.*YStd+YMean);
lower=-pi*ones(1,6);
upper=pi*ones(1,6);
margin=1e-5;
YPred=max(YPred,lower+margin);
YPred=min(YPred,upper-margin);
NTest=size(XTest,1);
positionErrorAI=zeros(NTest,1);
positionErrorHybrid=zeros(NTest,1);
success=false(NTest,1);
for i=1:NTest
    targetPosition=XTest(i,1:3);
    quat=XTest(i,4:7);
    targetPose=trvec2tform(targetPosition)*quat2tform(quat);
    aiConfig=homeConfiguration(robot);
    for j=1:6
        aiConfig(j).JointPosition=YPred(i,j);
    end
    TAI=getTransform(robot,aiConfig,"jaw6_1",robot.BaseName);
    positionErrorAI(i)=norm(tform2trvec(TAI)-targetPosition);
    try
        [refinedConfig,~]=ik("jaw6_1",targetPose,weights,aiConfig);
        TRefined=getTransform(robot,refinedConfig,"jaw6_1",robot.BaseName);
        positionErrorHybrid(i)=norm(tform2trvec(TRefined)-targetPosition);
        success(i)=true;
    catch
        positionErrorHybrid(i)=NaN;
    end
end
valid=success;
aiError=positionErrorAI(valid);
hybridError=positionErrorHybrid(valid);
fprintf("Successful IK solutions: %d / %d\n",sum(valid),NTest);
fprintf("AI only mean: %.2f mm\n",mean(aiError)*1000);
fprintf("AI only median: %.2f mm\n",median(aiError)*1000);
fprintf("AI only max: %.2f mm\n",max(aiError)*1000);
fprintf("Hybrid mean: %.4f mm\n",mean(hybridError)*1000);
fprintf("Hybrid median: %.4f mm\n",median(hybridError)*1000);
fprintf("Hybrid max: %.4f mm\n",max(hybridError)*1000);
fprintf("Hybrid within 15 cm: %.2f %%\n",mean(hybridError<=0.15)*100);
save("hybrid_ik_results.mat","positionErrorAI","positionErrorHybrid","success");