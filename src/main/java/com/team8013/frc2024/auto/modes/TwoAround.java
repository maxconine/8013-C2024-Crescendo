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
import com.team8013.frc2024.shuffleboard.ShuffleBoardInteractions;
import com.team8013.frc2024.subsystems.Drive;
import com.team8013.frc2024.subsystems.Superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class TwoAround extends AutoModeBase {

    private Superstructure mSuperstructure;

    // required PathWeaver trajectory paths
    String path_A = "paths/2024Paths/DriveAroundMiddle.path";
    String path_B = "paths/2024Paths/DriveToStageNote_B.path";
    String path_C = "paths/2024Paths/DriveToStageNote_C.path";

    // trajectories
    SwerveTrajectoryAction driveToFirstNote_A;
    final Trajectory drive_to_first_note_path_A;

    // SwerveTrajectoryAction driveToFirstNote_B;
    // final Trajectory drive_to_first_note_path_B;

    // SwerveTrajectoryAction driveToFirstNote_C;
    // final Trajectory drive_to_first_note_path_C;

    public TwoAround() {
        mSuperstructure = Superstructure.getInstance();

        // read trajectories from PathWeaver and generate trajectory actions
        drive_to_first_note_path_A = AutoTrajectoryReader.generateTrajectoryFromFile(path_A,
                Constants.AutoConstants.createConfig(1.2, 1.5, 0.0, 0));
        driveToFirstNote_A = new SwerveTrajectoryAction(drive_to_first_note_path_A, Rotation2d.fromDegrees(180));
        ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drive_to_first_note_path_A);

        // drive_to_first_note_path_B = AutoTrajectoryReader.generateTrajectoryFromFile(path_B,
        // Constants.AutoConstants.createConfig(1, 1.5, 0.0, 0));
        // driveToFirstNote_B = new SwerveTrajectoryAction(drive_to_first_note_path_B, Rotation2d.fromDegrees(180));
        // ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drive_to_first_note_path_B);

        // drive_to_first_note_path_C = AutoTrajectoryReader.generateTrajectoryFromFile(path_C,
        // Constants.AutoConstants.createConfig(1, 1.5, 0.0, 0));
        // driveToFirstNote_C = new SwerveTrajectoryAction(drive_to_first_note_path_C, Rotation2d.fromDegrees(180));
        // ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drive_to_first_note_path_C);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new LambdaAction(() -> Drive.getInstance().resetOdometry(getStartingPose())));

        System.out.println("Running 2 note auto");
        mSuperstructure.autoShot();
        runAction(new WaitAction(1.2));
        // mSuperstructure.setSuperstuctureIntakingGround();
        // mSuperstructure.setSuperstuctureTransferToShooter();
        // mSuperstructure.setSuperstuctureShoot(true);

        runAction(new ParallelAction(List.of(
            driveToFirstNote_A,
            new SeriesAction(List.of(
                new LambdaAction(() -> mSuperstructure.setSuperstuctureIntakingGround()),
                new LambdaAction(() -> Drive.getInstance()
                        .setAutoHeading(Rotation2d.fromDegrees(180.0))),
                new WaitToPassXCoordinateAction(15.5),
                new LambdaAction(() -> Drive.getInstance()
                        .setAutoHeading(Rotation2d.fromDegrees(130.0))),
                new WaitAction(0.8),
                new WaitToPassXCoordinateAction(15),
                new LambdaAction(() -> Drive.getInstance()
                        .setAutoHeading(Rotation2d.fromDegrees(180.0))),
                new WaitAction(0.3),
                new LambdaAction(() -> mSuperstructure.setSuperstuctureStow()),
                new LambdaAction(() -> mSuperstructure.autoShot()))))));

        // runAction(new ParallelAction(List.of(
        //         driveToFirstNote,
        //         new LambdaAction(() -> Drive.getInstance()
        //                         .setAutoHeading(Rotation2d.fromDegrees(0))))));

        // mSuperstructure.setSuperstuctureIntakingGround();

        // mSuperstructure.stowState();
        // runAction(new WaitForSuperstructureAction());
        // System.out.println("Finished waiting for stow");
        // mSuperstructure.scoreL3State();

    }

    @Override
    public Pose2d getStartingPose() {
        Rotation2d startingRotation = Rotation2d.fromDegrees(180.0);
        if (Robot.is_red_alliance) {
            startingRotation = Rotation2d.fromDegrees(0.0);
        }
        return new Pose2d(drive_to_first_note_path_A.getInitialPose().getTranslation(), startingRotation);
    }
}
