package com.team8013.frc2024.auto.modes;

import com.team8013.frc2024.Robot;
import com.team8013.frc2024.auto.AutoModeBase;
import com.team8013.frc2024.auto.AutoModeEndedException;
import com.team8013.frc2024.auto.actions.LambdaAction;
import com.team8013.frc2024.auto.actions.WaitForLimelightNotePickup;
import com.team8013.frc2024.subsystems.Drive;
import com.team8013.frc2024.subsystems.Limelight;
import com.team8013.frc2024.subsystems.Superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TestAutoNotePickup extends AutoModeBase {

    private Superstructure mSuperstructure;
    private Limelight mLimelight;

    public TestAutoNotePickup() {
        mSuperstructure = Superstructure.getInstance();
        mLimelight = Limelight.getInstance();

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new LambdaAction(() -> Drive.getInstance().resetOdometry(getStartingPose())));

        System.out.println("Running 1 note auto pickup");
        mSuperstructure.setSuperstuctureIntakingGround();
        mLimelight.wantNoteChase(true);
        runAction(new WaitForLimelightNotePickup());
        mSuperstructure.setSuperstuctureStow();

    }

    @Override
    public Pose2d getStartingPose() {
        Rotation2d startingRotation = Rotation2d.fromDegrees(180.0);
        if (Robot.is_red_alliance) {
            startingRotation = Rotation2d.fromDegrees(0.0);
        }
        return new Pose2d(0,0, startingRotation);
    }
}
