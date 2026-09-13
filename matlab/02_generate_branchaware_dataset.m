clear; clc;
robot = importrobot("../robot/base_fixed.urdf");
robot.DataFormat = "struct";
ik = inverseKinematics("RigidBodyTree",robot);
weights = [1 1 1 1 1 1];
N = 50000;
X = zeros(N,13);
Y = zeros(N,6);
currentConfig = homeConfiguration(robot);
lower = -pi*ones(1,6);
upper = pi*ones(1,6);
margin = 1e-5;
validCount = 0;
attempts = 0;
while validCount < N
    attempts = attempts+1;
    qRandom = randomConfiguration(robot);
    T = getTransform(robot,qRandom,"jaw6_1",robot.BaseName);
    position = tform2trvec(T);
    quat = tform2quat(T);
    if quat(1) < 0, quat = -quat; end
    qPrevious = [currentConfig(1:6).JointPosition];
    [configSol,~] = ik("jaw6_1",T,weights,currentConfig);
    qSolution = [configSol(1:6).JointPosition];
    validLimits = all(qSolution > lower+margin & qSolution < upper-margin);
    if validLimits
        Tcheck = getTransform(robot,configSol,"jaw6_1",robot.BaseName);
        positionError = norm(tform2trvec(Tcheck)-position);
        validPose = positionError < 1e-4;
    else
        validPose = false;
    end
    if validLimits && validPose
        validCount = validCount+1;
        X(validCount,:) = [position quat qPrevious];
        Y(validCount,:) = qSolution;
        currentConfig = configSol;
    end
end
save("ik_dataset_branchaware.mat","X","Y");