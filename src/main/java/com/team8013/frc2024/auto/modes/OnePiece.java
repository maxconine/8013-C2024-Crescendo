// package com.team8013.frc2024.auto.modes;

// import com.team8013.frc2024.Constants;
// import com.team8013.frc2024.Robot;
// import com.team8013.frc2024.auto.AutoModeBase;
// import com.team8013.frc2024.auto.AutoModeEndedException;
// import com.team8013.frc2024.auto.AutoTrajectoryReader;
// import com.team8013.frc2024.auto.actions.LambdaAction;
// import com.team8013.frc2024.auto.actions.SwerveTrajectoryAction;
// import com.team8013.frc2024.shuffleboard.ShuffleBoardInteractions;
// import com.team8013.frc2024.subsystems.Drive;
// import com.team8013.frc2024.subsystems.Superstructure;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.trajectory.Trajectory;

// public class OnePiece extends AutoModeBase {

//     private Superstructure mSuperstructure;

//     // required PathWeaver trajectory paths
//     String path = "paths/OnePlusBalanceDeploy.path";

//     // trajectories
//     SwerveTrajectoryAction driveToChargeStation;
//     final Trajectory drive_to_charge_station_path;

//     public OnePiece() {
//         mSuperstructure = Superstructure.getInstance();
        
//         // read trajectories from PathWeaver and generate trajectory actions
//         drive_to_charge_station_path = AutoTrajectoryReader.generateTrajectoryFromFile(path,
//                 Constants.AutoConstants.createConfig(2.0, 10.0, 0.0, 0.0));
//         driveToChargeStation = new SwerveTrajectoryAction(drive_to_charge_station_path, Rotation2d.fromDegrees(0.0));
//         ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drive_to_charge_station_path);
//     }

//     @Override
//     protected void routine() throws AutoModeEndedException {
//         runAction(new LambdaAction(() -> Drive.getInstance().resetOdometry(getStartingPose())));

//         // Debug
//         // System.out.println("Running One Plus Balance");
//         // mSuperstructure.stowState();
//         // runAction(new WaitForSuperstructureAction());
//         // System.out.println("Finished waiting for stow");
//         // mSuperstructure.scoreL3State();
//         // runAction(new WaitForSuperstructureAction());
//         // System.out.println("Finished waiting for extend");
//         // runAction(new WaitAction(0.5));
//         // mEffector.setState(State.OUTTAKING_CONE);
//         // runAction(new WaitAction(0.5));
//         // mEffector.setState(State.IDLE);
//         // System.out.println("Finished running endeffector");
//         // mSuperstructure.stowElevator();
//         // runAction(new WaitForSuperstructureAction());
//         runAction(driveToChargeStation);

//     }

//     @Override
//     public Pose2d getStartingPose() {
//         Rotation2d startingRotation = Rotation2d.fromDegrees(180.0);
//         if (Robot.is_red_alliance) {
//             startingRotation = Rotation2d.fromDegrees(0.0);
//         }
//         return new Pose2d(drive_to_charge_station_path.getInitialPose().getTranslation(), startingRotation);
//     }
// }
