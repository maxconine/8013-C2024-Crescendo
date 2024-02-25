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

public class TwoRightRed extends AutoModeBase {

    private Superstructure mSuperstructure;

    // required PathWeaver trajectory paths
    String path_A = "paths/2024Paths/RightRed_A.path";
    String path_B = "paths/2024Paths/RightRed_B.path";

    // trajectories
    SwerveTrajectoryAction driveToFirstNote;
    final Trajectory drivePath_A;

    SwerveTrajectoryAction driveToShootFirstNote;
    final Trajectory drivePath_B;


    public TwoRightRed() {
        mSuperstructure = Superstructure.getInstance();

        // read trajectories from PathWeaver and generate trajectory actions
        drivePath_A = AutoTrajectoryReader.generateTrajectoryFromFile(path_A,
                Constants.AutoConstants.createConfig(0.85, 1.5, 0.0, 0));
        driveToFirstNote = new SwerveTrajectoryAction(drivePath_A, Rotation2d.fromDegrees(180));
        ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drivePath_A);

        drivePath_B = AutoTrajectoryReader.generateTrajectoryFromFile(path_B,
                Constants.AutoConstants.createConfig(0.85, 1.5, 0.0, 0));
        driveToShootFirstNote = new SwerveTrajectoryAction(drivePath_B, Rotation2d.fromDegrees(180));
        ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drivePath_B);

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new LambdaAction(() -> Drive.getInstance().resetOdometry(getStartingPose())));

        System.out.println("Running 2 note RED RIGHT auto");
        mSuperstructure.autoShot();
        runAction(new WaitAction(1.2));

        runAction(new ParallelAction(List.of(
                driveToFirstNote,
                new SeriesAction(List.of(
                        new WaitToPassXCoordinateAction(15.8),
                        new LambdaAction(() -> Drive.getInstance()
                                .setAutoHeading(Rotation2d.fromDegrees(0.0))),
                        new WaitAction(0.1),
                        new LambdaAction(() -> mSuperstructure.setSuperstuctureIntakingGround()))))));

        runAction(new ParallelAction(List.of(
                driveToShootFirstNote,
                new SeriesAction(List.of(
                        // new WaitToPassXCoordinateAction(3.2),
                        new LambdaAction(() -> Drive.getInstance()
                                .setAutoHeading(Rotation2d.fromDegrees(120.0))),
                        new WaitToPassXCoordinateAction(14.7),
                        new LambdaAction(() -> mSuperstructure.setSuperstuctureTransferToShooter()),
                        new WaitToPassXCoordinateAction(15.6),
                        new LambdaAction(() -> mSuperstructure.autoShot()))))));


    }

    @Override
    public Pose2d getStartingPose() {
        Rotation2d startingRotation = Rotation2d.fromDegrees(120.0);
        if (Robot.is_red_alliance) {
            startingRotation = Rotation2d.fromDegrees(60);
        }
        return new Pose2d(drivePath_A.getInitialPose().getTranslation(), startingRotation);
    }
}
