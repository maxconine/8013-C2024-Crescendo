package com.team8013.frc2024.auto.actions;

import com.team8013.frc2024.subsystems.Drive;

public class WaitForHeadingAction implements Action {
    Drive drive;
    double lowThreshold;
    double highThreshold;

    public WaitForHeadingAction(double lowThreshold, double highThreshold) {
        drive = Drive.getInstance();
        this.lowThreshold = lowThreshold;
        this.highThreshold = highThreshold;
    }

    @Override
    public boolean isFinished() {
        double heading = drive.getPose().getRotation().getDegrees();
        System.out.println(heading);
        return heading >= lowThreshold && heading <= highThreshold;
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
