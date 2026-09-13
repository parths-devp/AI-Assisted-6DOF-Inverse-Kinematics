# AI-Assisted 6-DOF Inverse Kinematics

A MATLAB simulation project for a 6-DOF articulated robot designed in Fusion 360 and exported to URDF.

The final system uses a hybrid AI-assisted IK pipeline: a neural network predicts a configuration-dependent initial joint-angle guess, and MATLAB numerical inverse kinematics refines that guess to reach the requested end-effector pose.

## Pipeline

```text
Desired pose + previous joint configuration
                  ↓
        Branch-aware neural network
                  ↓
          AI joint-angle guess
                  ↓
        MATLAB numerical IK solver
                  ↓
          Refined joint solution
                  ↓
                Robot
```

The project is intentionally described as **AI-assisted/hybrid IK**, rather than standalone AI IK, because the numerical IK solver performs the final refinement.

## Robot model

The Fusion 360 assembly contains a serial 6-DOF arm plus two gripper-finger joints. The first six revolute joints form the arm: Revolute6 through Revolute11. The gripper fingers use Revolute14 and Revolute15.

MATLAB uses `Base_1` as the base frame and `jaw6_1` as the end-effector frame.

## Why branch-aware IK?

Initial direct pose-to-joint regression performed poorly because inverse kinematics can have multiple valid joint configurations for the same end-effector pose. Euler-angle representations also have discontinuities.

The final neural network receives 13 inputs:

```text
[x y z qw qx qy qz q1_prev q2_prev q3_prev q4_prev q5_prev q6_prev]
```

and predicts six arm joint angles:

```text
[q1 q2 q3 q4 q5 q6]
```

The previous joint configuration provides information about the current IK branch.

## Dataset

50,000 valid samples were generated using MATLAB's numerical `inverseKinematics` solver. Each sample contains a target pose, the previous arm configuration, and a valid numerical IK solution.

Dataset split: 35,000 training, 7,500 validation, and 7,500 test samples. Inputs and outputs are standardized using statistics computed only from the training set.

Large `.mat` datasets and trained network files are kept out of the initial repository to avoid unnecessarily large Git history. They can be generated locally from the MATLAB workflow.

## Neural network

Final architecture:

```text
13 → 128 → 256 → 256 → 128 → 64 → 6
```

ReLU activations are used between the fully connected layers. Training uses Adam with a learning rate of `0.001`, mini-batch size `256`, and up to `100` epochs.

## Final demonstration

The final demonstration commands the end effector along a closed 10 cm × 10 cm square in the XY plane at `Z = 0.60 m`, while maintaining a fixed orientation. The trajectory contains 157 Cartesian points.

Final simulated results:

| Metric | Result |
|---|---:|
| Mean position error | 0.0000 mm |
| Median position error | 0.0000 mm |
| Maximum position error | 0.0000 mm |
| Within 5 cm | 100% |
| Within 10 cm | 100% |
| Within 15 cm | 100% |

These values are from the MATLAB robot model and FK/IK simulation; they are not measurements of a physical robot.

For comparison, the AI-only prediction on the square trajectory had a mean Cartesian error of about 369 mm. The numerical IK refinement is essential for the final accuracy.

## Runtime

In a 500-sample comparison, MATLAB IK from a home configuration averaged about 10.0 ms/sample, while AI-initialized numerical IK averaged about 9.0 ms/sample overall (about 1.11× speedup). The speed improvement is modest; the main role of the network is to provide a configuration-dependent initial guess and help select a consistent IK branch.

## Requirements

- MATLAB R2026a or compatible recent MATLAB release
- Robotics System Toolbox
- Deep Learning Toolbox

## Repository structure

```text
AI-Assisted-6DOF-Inverse-Kinematics/
├── README.md
├── requirements.md
├── .gitignore
├── matlab/
│   └── final_square_demo.m
├── robot/
│   ├── base_fixed.urdf
│   └── meshes/
├── dataset/
└── results/
```

## URDF note

`base_fixed.urdf` references STL meshes under `robot/meshes/`. The mesh binaries are not included in this initial commit because the available project files did not contain the STL files. Add the exported Fusion 360 STL files using the exact names referenced by the URDF to restore the complete visual/collision model.

## Workflow

1. Import the URDF with `importrobot`.
2. Verify the robot joints and limits.
3. Test forward kinematics with `getTransform`.
4. Test numerical inverse kinematics with `inverseKinematics`.
5. Generate the branch-aware dataset.
6. Prepare and normalize the dataset.
7. Train the neural network.
8. Use the network output as the numerical IK initial guess.
9. Generate Cartesian trajectories.
10. Evaluate and demonstrate the final square trajectory.
