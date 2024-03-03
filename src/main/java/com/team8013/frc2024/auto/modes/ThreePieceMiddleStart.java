package com.team8013.frc2024.auto.modes;

import java.util.List;

import com.team8013.frc2024.Constants;
import com.team8013.frc2024.Robot;
import com.team8013.frc2024.auto.AutoModeBase;
import com.team8013.frc2024.auto.AutoModeEndedException;
import com.team8013.frc2024.auto.AutoTrajectoryReader;
import com.team8013.frc2024.auto.actions.LambdaAction;
import com.team8013.frc2024.auto.actions.ParallelAction;
import com.team8013.frc2024.auto.actions.SeriesAction;
import com.team8013.frc2024.auto.actions.SwerveTrajectoryAction;
import com.team8013.frc2024.auto.actions.WaitAction;
import com.team8013.frc2024.auto.actions.WaitToPassXCoordinateAction;
import com.team8013.frc2024.auto.actions.WaitToPassYCoordinateAction;
import com.team8013.frc2024.shuffleboard.ShuffleBoardInteractions;
import com.team8013.frc2024.subsystems.Drive;
import com.team8013.frc2024.subsystems.Superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class ThreePieceMiddleStart extends AutoModeBase {

        private Superstructure mSuperstructure;

        // required PathWeaver trajectory paths
        String path_A = "paths/2024Paths/3PieceMiddleStart_C.path";
        String path_B = "paths/2024Paths/3PieceMiddleStart_D.path";
        String path_C = "paths/2024Paths/3PieceMiddleStart_A.path";
        String path_D = "paths/2024Paths/3PieceMiddleStart_B.path";
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

        public ThreePieceMiddleStart() {
                mSuperstructure = Superstructure.getInstance();

                // read trajectories from PathWeaver and generate trajectory actions
                drivePath_A = AutoTrajectoryReader.generateTrajectoryFromFile(path_A,
                                Constants.AutoConstants.createConfig(0.85, 1.5, 0.0, 0));
                driveToFirstNote = new SwerveTrajectoryAction(drivePath_A, Rotation2d.fromDegrees(180));
                ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drivePath_A);

                drivePath_B = AutoTrajectoryReader.generateTrajectoryFromFile(path_B,
                                Constants.AutoConstants.createConfig(1.1, 1.5, 0.0, 0));
                driveToShootFirstNote = new SwerveTrajectoryAction(drivePath_B, Rotation2d.fromDegrees(0));
                ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drivePath_B);

                drivePath_C = AutoTrajectoryReader.generateTrajectoryFromFile(path_C,
                                Constants.AutoConstants.createConfig(0.7, 1.5, 0.0, 0));
                driveToPickupSecondNote = new SwerveTrajectoryAction(drivePath_C, Rotation2d.fromDegrees(0));
                ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drivePath_C);

                drivePath_D = AutoTrajectoryReader.generateTrajectoryFromFile(path_D,
                                Constants.AutoConstants.createConfig(1.6, 1.5, 0.0, 0));
                driveToShootSecondNote = new SwerveTrajectoryAction(drivePath_D, Rotation2d.fromDegrees(180));
                ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drivePath_D);

                drivePath_E = AutoTrajectoryReader.generateTrajectoryFromFile(path_E,
                                Constants.AutoConstants.createConfig(3, 1.5, 0.0, 0));
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
                runAction(new WaitAction(1.2));
                mSuperstructure.setSuperstuctureStow();

                runAction(new ParallelAction(List.of(
                                driveToFirstNote,
                                new SeriesAction(List.of(
                                                new WaitToPassYCoordinateAction(5.25),
                                                new LambdaAction(() -> Drive.getInstance()
                                                                .setAutoHeading(Rotation2d.fromDegrees(0.0))),
                                                new WaitAction(0.1),
                                                new LambdaAction(() -> mSuperstructure
                                                                .setSuperstuctureIntakingGround()))))));

                mSuperstructure.setSuperstuctureStow();

                runAction(new ParallelAction(List.of(
                                driveToShootFirstNote,
                                new SeriesAction(List.of(
                                                new WaitAction(0.3),
                                                new LambdaAction(() -> Drive.getInstance()
                                                                .setAutoHeading(Rotation2d.fromDegrees(180.0))),
                                                new WaitToPassXCoordinateAction(14.7),
                                                new LambdaAction(() -> mSuperstructure
                                                                .setSuperstuctureTransferToShooter()),
                                                new WaitToPassXCoordinateAction(15.6),
                                                new LambdaAction(() -> mSuperstructure.autoShot()))))));

                runAction(new WaitAction(0.45));
                mSuperstructure.setSuperstuctureStow();

                runAction(new ParallelAction(List.of(
                                driveToPickupSecondNote,
                                new SeriesAction(List.of(
                                                new WaitToPassXCoordinateAction(15.57),
                                                new LambdaAction(
                                                                () -> mSuperstructure.setSuperstuctureIntakingGround()),
                                                new LambdaAction(() -> Drive.getInstance()
                                                                .setAutoHeading(Rotation2d.fromDegrees(0.0))))))));

                mSuperstructure.setSuperstuctureStow();

                runAction(new ParallelAction(List.of(
                                driveToShootSecondNote,
                                new SeriesAction(List.of(
                                                // new WaitToPassXCoordinateAction(3.2),
                                                new LambdaAction(() -> Drive.getInstance()
                                                                .setAutoHeading(Rotation2d.fromDegrees(180.0))),
                                                new WaitToPassXCoordinateAction(15.4),
                                                new LambdaAction(() -> mSuperstructure
                                                                .setSuperstuctureTransferToShooter()),
                                                new WaitToPassXCoordinateAction(15.6),
                                                new LambdaAction(() -> mSuperstructure.autoShot()))))));

                runAction(new WaitAction(0.8));
                mSuperstructure.setSuperstuctureStow();

                runAction(new ParallelAction(List.of(
                                driveOut,
                                new SeriesAction(List.of(
                                                new WaitAction(0.3),
                                                new LambdaAction(() -> Drive.getInstance()
                                                                .setAutoHeading(Rotation2d.fromDegrees(0))))))));
                // runAction(new ParallelAction(List.of(
                // driveToFirstNote,
                // new LambdaAction(() -> Drive.getInstance()
                // .setAutoHeading(Rotation2d.fromDegrees(0))))));

                // mSuperstructure.setSuperstuctureIntakingGround();

                // mSuperstructure.stowState();
                // runAction(new WaitForSuperstructureAction());
                // System.out.println("Finished waiting for stow");
                // mSuperstructure.scoreL3State();

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
