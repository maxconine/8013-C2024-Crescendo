package com.team8013.frc2024.auto.modes;

import java.util.List;

import com.team8013.frc2024.Constants;
import com.team8013.frc2024.Robot;
import com.team8013.frc2024.auto.AutoModeBase;
import com.team8013.frc2024.auto.AutoModeEndedException;
import com.team8013.frc2024.auto.AutoTrajectoryReader;
import com.team8013.frc2024.auto.actions.LambdaAction;
import com.team8013.frc2024.auto.actions.WaitAction;
import com.team8013.frc2024.auto.actions.ParallelAction;
import com.team8013.frc2024.auto.actions.SeriesAction;
import com.team8013.frc2024.auto.actions.SwerveTrajectoryAction;
import com.team8013.frc2024.auto.actions.WaitAction;
import com.team8013.frc2024.auto.actions.WaitToPassXCoordinateAction;
import com.team8013.frc2024.controlboard.ControlBoard;
import com.team8013.frc2024.shuffleboard.ShuffleBoardInteractions;
import com.team8013.frc2024.subsystems.Drive;
import com.team8013.frc2024.subsystems.Superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class ThreePieceMiddleStartAmpSide extends AutoModeBase {

        private Superstructure mSuperstructure;
        private ControlBoard mControlBoard;

        // required PathWeaver trajectory paths
        String path_A = "paths/2024Paths/TwoMiddleSmooth.path";
        String path_B = "paths/2024Paths/TwoMiddle_B.path";
        String path_C = "paths/2024Paths/3PieceMiddleStartAmpSide_C.path";
        String path_D = "paths/2024Paths/3PieceMiddleStartAmpSide_D.path";
        String path_E = "paths/2024Paths/TwoMiddle_C.path";

        // trajectories
        SwerveTrajectoryAction driveToFirstNote;
        final Trajectory drivePath_A;

        SwerveTrajectoryAction driveToShootFirstNote;
        final Trajectory drivePath_B;

        SwerveTrajectoryAction driveToPickupSecondNote;
        final Trajectory drivePath_C;

        SwerveTrajectoryAction driveToShootSecondNote;
        final Trajectory drivePath_D;

        SwerveTrajectoryAction driveOut;
        final Trajectory drivePath_E;

        public ThreePieceMiddleStartAmpSide() {
                mSuperstructure = Superstructure.getInstance();
                mControlBoard = ControlBoard.getInstance();

                // read trajectories from PathWeaver and generate trajectory actions
                drivePath_A = AutoTrajectoryReader.generateTrajectoryFromFile(path_A,
                                Constants.AutoConstants.createConfig(0.7, 1.2, 0.0, 0));
                driveToFirstNote = new SwerveTrajectoryAction(drivePath_A, Rotation2d.fromDegrees(180));
                ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drivePath_A);

                drivePath_B = AutoTrajectoryReader.generateTrajectoryFromFile(path_B,
                                Constants.AutoConstants.createConfig(0.8, 1.2, 0.0, 0));
                driveToShootFirstNote = new SwerveTrajectoryAction(drivePath_B, Rotation2d.fromDegrees(0));
                ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drivePath_B);

                drivePath_C = AutoTrajectoryReader.generateTrajectoryFromFile(path_C,
                                Constants.AutoConstants.createConfig(1.3, 1, 0.0, 0));
                driveToPickupSecondNote = new SwerveTrajectoryAction(drivePath_C, Rotation2d.fromDegrees(180));
                ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drivePath_C);

                drivePath_D = AutoTrajectoryReader.generateTrajectoryFromFile(path_D,
                                Constants.AutoConstants.createConfig(1.5, 1.2, 0.0, 0));
                driveToShootSecondNote = new SwerveTrajectoryAction(drivePath_D, Rotation2d.fromDegrees(270));
                ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drivePath_D);

                drivePath_E = AutoTrajectoryReader.generateTrajectoryFromFile(path_E,
                                Constants.AutoConstants.createConfig(5, 2.3, 0.0, 0));
                driveOut = new SwerveTrajectoryAction(drivePath_E, Rotation2d.fromDegrees(180));
                ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drivePath_E);
        }

        @Override
        protected void routine() throws AutoModeEndedException {
                runAction(new LambdaAction(() -> Drive.getInstance().resetOdometry(getStartingPose())));

                System.out.println("Running 3 note auto");
                // mSuperstructure.setSuperstuctureTransferToShooter();
                // runAction(new WaitAction(0.2));
                mSuperstructure.autoShot();
                runAction(new WaitAction(1.4));
                mSuperstructure.disableAutoShot();
                mSuperstructure.setSuperstuctureStow();

                runAction(new ParallelAction(List.of(
                                driveToFirstNote,
                                new SeriesAction(List.of(
                                                // new WaitToPassXCoordinateAction(15.62),
                                                new WaitAction(0.1),
                                                new LambdaAction(() -> Drive.getInstance()
                                                                .setAutoHeading(Rotation2d.fromDegrees(-2))),
                                                // new WaitForHeadingAction(160,200),
                                                new WaitAction(0.25), // used to be 0.15 before wpi
                                                new LambdaAction(() -> mSuperstructure
                                                                .setSuperstuctureIntakingGround()),
                                                new WaitAction(1.6), // used to be 1.5 before wpi
                                                new LambdaAction(() -> mSuperstructure
                                                .setSuperstuctureStow()),
                                                new WaitAction(0.05),
                                                new LambdaAction(() -> Drive.getInstance()
                                                                .setAutoHeading(Rotation2d.fromDegrees(180))),
                                                new WaitAction(0.8),
                                                new LambdaAction(() -> mSuperstructure
                                                                .setSuperstuctureTransferToShooter()))))));

                // runAction(new ParallelAction(List.of(
                // driveToFirstNote,
                // new SeriesAction(List.of(
                // // new WaitToPassXCoordinateAction(15.62),
                // new WaitAction(0.1),
                // new LambdaAction(() -> Drive.getInstance()
                // .setAutoHeading(Rotation2d.fromDegrees(-2))),
                // // new WaitForHeadingAction(160,200),
                // new WaitAction(0.15),
                // new LambdaAction(() -> mSuperstructure
                // .setSuperstuctureIntakingGround()))))));

                // mSuperstructure.setSuperstuctureStow();

                // runAction(new ParallelAction(List.of(
                // driveToShootFirstNote,
                // new SeriesAction(List.of(
                // new WaitAction(0.05),
                // new LambdaAction(() -> Drive.getInstance()
                // .setAutoHeading(Rotation2d.fromDegrees(180))),
                // new WaitAction(0.3),
                // new LambdaAction(() -> mSuperstructure
                // .setSuperstuctureTransferToShooter()))))));
                mSuperstructure.autoShot();
                runAction(new WaitAction(0.5)); // used to use 0.4 before wpi
                mSuperstructure.setSuperstuctureShoot(false);
                mSuperstructure.disableAutoShot();
                mSuperstructure.setSuperstuctureStow();

                runAction(new ParallelAction(List.of(
                                driveToPickupSecondNote,
                                new SeriesAction(List.of(
                                                new WaitAction(0.1),
                                                new LambdaAction(() -> Drive.getInstance()
                                                                .setAutoHeading(Rotation2d.fromDegrees(270))),
                                                new WaitAction(0.3),
                                                new LambdaAction(
                                                                () -> mSuperstructure
                                                                                .setSuperstuctureIntakingGround()))))));

                //mSuperstructure.setSuperstuctureStow();
                mSuperstructure.setSuperstuctureShoot(false);

                runAction(new ParallelAction(List.of(
                                driveToShootSecondNote,
                                new SeriesAction(List.of(
                                                new WaitAction(0.05),
                                                new LambdaAction((() -> mSuperstructure
                                                                .setSuperstuctureStow())),
                                                new LambdaAction(() -> Drive.getInstance()
                                                                .setAutoHeading(Rotation2d.fromDegrees(180.0))),
                                                new WaitAction(0.8),
                                                new LambdaAction(() -> mSuperstructure
                                                                .setSuperstuctureTransferToShooter()))))));
                mSuperstructure.autoShot();
                runAction(new WaitAction(0.5));
                mSuperstructure.setSuperstuctureStow();
                mSuperstructure.disableAutoShot();

                runAction(new ParallelAction(List.of(
                                driveOut,
                                new SeriesAction(List.of(
                                                // new WaitAction(0.1),
                                                // new LambdaAction(() -> Drive.getInstance()
                                                // .setAutoHeading(Rotation2d.fromDegrees(-2))),
                                                // new WaitAction(3),
                                                new WaitToPassXCoordinateAction(13),
                                                new LambdaAction((() -> mSuperstructure
                                                                .setSuperstuctureIntakingGround())),
                                                new WaitAction(0.1),
                                                new LambdaAction(() -> Drive.getInstance()
                                                                .setAutoHeading(Rotation2d.fromDegrees(-145))),
                                                new WaitAction(1.9),
                                                new LambdaAction(() -> mSuperstructure
                                                                .setSuperstuctureStow()),
                                                new LambdaAction(() -> Drive.getInstance()
                                                                .setAutoHeading(Rotation2d.fromDegrees(180))),
                                                new WaitToPassXCoordinateAction(13),
                                                new LambdaAction(() -> mSuperstructure
                                                                .setSuperstuctureTransferToShooter()),
                                                new WaitAction(0.2),
                                                new LambdaAction(() -> mControlBoard.setAutoSnapToTarget(true)))))));

                mControlBoard.setAutoSnapToTarget(true);
                runAction(new WaitAction(0.1));
                mSuperstructure.autoShot();

        }

        @Override
        public Pose2d getStartingPose() {
                Rotation2d startingRotation = Rotation2d.fromDegrees(180);// TODO: NO IDEA IF THIS IS RIGHT
                if (Robot.is_red_alliance) {
                        startingRotation = Rotation2d.fromDegrees(0);
                }
                return new Pose2d(drivePath_A.getInitialPose().getTranslation(), startingRotation);
        }
}
