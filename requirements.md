# Requirements

## Software

- MATLAB R2026a (or a compatible recent release)
- Robotics System Toolbox
- Deep Learning Toolbox

## Robot model

The project uses a 6-DOF serial manipulator exported from Fusion 360 as URDF.

- Base frame: `Base_1`
- End-effector frame: `jaw6_1`
- Arm joints: Revolute6–Revolute11
- Gripper joints: Revolute14–Revolute15

## Main MATLAB functions used

- `importrobot`
- `show`
- `getTransform`
- `inverseKinematics`
- `randomConfiguration`
- `trapveltraj`
- `trvec2tform`
- `quat2tform`
- `tform2trvec`
- `tform2quat`
- `eul2tform`

## Project files

Large generated datasets (`.mat`) and trained neural-network files are intentionally excluded from the repository. They can be regenerated using the project workflow.

The URDF expects the corresponding Fusion 360 exported STL mesh files in `robot/meshes/`.
