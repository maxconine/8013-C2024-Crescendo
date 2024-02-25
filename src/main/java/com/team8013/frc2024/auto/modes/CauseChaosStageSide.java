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
import com.team8013.frc2024.auto.actions.WaitForSuperstructureAction;
import com.team8013.frc2024.auto.actions.WaitToPassXCoordinateAction;
import com.team8013.frc2024.shuffleboard.ShuffleBoardInteractions;
import com.team8013.frc2024.subsystems.Drive;
import com.team8013.frc2024.subsystems.Superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class CauseChaosStageSide extends AutoModeBase {

    private Superstructure mSuperstructure;

    // required PathWeaver trajectory paths
    String path = "paths/2024Paths/Cause_Chaos_StageSide.path";

    // trajectories
    SwerveTrajectoryAction driveToFirstNote;
    final Trajectory drive_to_first_note_path;

    public CauseChaosStageSide() {
        mSuperstructure = Superstructure.getInstance();

        // read trajectories from PathWeaver and generate trajectory actions
        drive_to_first_note_path = AutoTrajectoryReader.generateTrajectoryFromFile(path,
                Constants.AutoConstants.createConfig(4, 1.5, 0.0, 0.0));
        driveToFirstNote = new SwerveTrajectoryAction(drive_to_first_note_path, Rotation2d.fromDegrees(240.0));
        ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drive_to_first_note_path);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new LambdaAction(() -> Drive.getInstance().resetOdometry(getStartingPose())));

        System.out.println("Running cause chaos auto");
        mSuperstructure.autoShot();
        runAction(new WaitAction(1.2));
        mSuperstructure.setSuperstuctureStow();

        runAction(new ParallelAction(List.of(
                driveToFirstNote,
                new SeriesAction(List.of(
                        new WaitAction(3),
                        new LambdaAction(() -> Drive.getInstance()
                                .setAutoHeading(Rotation2d.fromDegrees(140))))))));
                        // new WaitAction(1),
                        // new LambdaAction(() -> Drive.getInstance()
                        //         .setAutoHeading(Rotation2d.fromDegrees(30))),
                        //         new WaitAction(1),
                        // new LambdaAction(() -> Drive.getInstance()
                        //         .setAutoHeading(Rotation2d.fromDegrees(230))),
                        //         new WaitAction(1),
                        // new LambdaAction(() -> Drive.getInstance()
                        //         .setAutoHeading(Rotation2d.fromDegrees(70))),
                        //         new WaitAction(1),
                        // new LambdaAction(() -> Drive.getInstance()
                        //         .setAutoHeading(Rotation2d.fromDegrees(270))),
                        //         new WaitAction(1),
                        // new LambdaAction(() -> Drive.getInstance()
                        //         .setAutoHeading(Rotation2d.fromDegrees(110))))));

        // mSuperstructure.setSuperstuctureIntakingGround();

        // mSuperstructure.stowState();
        // runAction(new WaitForSuperstructureAction());
        // System.out.println("Finished waiting for stow");
        // mSuperstructure.scoreL3State();

    }

    @Override
    public Pose2d getStartingPose() {
        Rotation2d startingRotation = Rotation2d.fromDegrees(240);
        if (Robot.is_red_alliance) {
            startingRotation = Rotation2d.fromDegrees(300);
        }
        return new Pose2d(drive_to_first_note_path.getInitialPose().getTranslation(), startingRotation);
    }
}
