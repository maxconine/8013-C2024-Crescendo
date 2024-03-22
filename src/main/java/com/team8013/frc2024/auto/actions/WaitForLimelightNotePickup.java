package com.team8013.frc2024.auto.actions;

import com.team8013.frc2024.subsystems.Limelight;

public class WaitForLimelightNotePickup implements Action {
    Limelight mLimelight;

    public WaitForLimelightNotePickup() {
        mLimelight = Limelight.getInstance();
    }

    @Override
    public boolean isFinished() {
        return mLimelight.isDoneWithNotePickup();
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

}
