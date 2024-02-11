package com.team8013.frc2024.auto.modes;

import com.team8013.frc2024.Constants;
import com.team8013.frc2024.Robot;
import com.team8013.frc2024.auto.AutoModeBase;
import com.team8013.frc2024.auto.AutoModeEndedException;
import com.team8013.frc2024.auto.AutoTrajectoryReader;
import com.team8013.frc2024.auto.actions.LambdaAction;
import com.team8013.frc2024.auto.actions.SwerveTrajectoryAction;
import com.team8013.frc2024.auto.actions.WaitForSuperstructureAction;
import com.team8013.frc2024.shuffleboard.ShuffleBoardInteractions;
import com.team8013.frc2024.subsystems.Drive;
import com.team8013.frc2024.subsystems.Superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class OneNote extends AutoModeBase {

    private Superstructure mSuperstructure;

    // required PathWeaver trajectory paths
    String path = "paths/driveToFirstNote.path";

    // trajectories
    SwerveTrajectoryAction driveToFirstNote;
    final Trajectory drive_to_first_note_path;

    public OneNote() {
        mSuperstructure = Superstructure.getInstance();
        
        // read trajectories from PathWeaver and generate trajectory actions
        drive_to_first_note_path = AutoTrajectoryReader.generateTrajectoryFromFile(path,
                Constants.AutoConstants.createConfig(2.0, 10.0, 0.0, 0.0));
        driveToFirstNote = new SwerveTrajectoryAction(drive_to_first_note_path, Rotation2d.fromDegrees(0.0));
        ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drive_to_first_note_path);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new LambdaAction(() -> Drive.getInstance().resetOdometry(getStartingPose())));

        System.out.println("Running 1 note auto");
        mSuperstructure.firstAutoShot();

        // mSuperstructure.stowState();
        // runAction(new WaitForSuperstructureAction());
        // System.out.println("Finished waiting for stow");
        // mSuperstructure.scoreL3State();

        runAction(driveToFirstNote);

    }

    @Override
    public Pose2d getStartingPose() {
        Rotation2d startingRotation = Rotation2d.fromDegrees(180.0);
        if (Robot.is_red_alliance) {
            startingRotation = Rotation2d.fromDegrees(0.0);
        }
        return new Pose2d(drive_to_first_note_path.getInitialPose().getTranslation(), startingRotation);
    }
}
