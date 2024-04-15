package com.team8013.frc2024.auto.actions;

import com.team8013.frc2024.subsystems.Superstructure;

import edu.wpi.first.wpilibj.Timer;

public class WaitForShootAction implements Action {
    private Superstructure superstructure;
    private Timer mTimer;
    private boolean timerStarted = false;

    public WaitForShootAction() {
        superstructure = Superstructure.getInstance();
    }

    @Override
    public boolean isFinished() {
        return mTimer.get()>0.2;
    }

    @Override
    public void start() {
        mTimer.reset();
        mTimer.stop();
        timerStarted = false;
    }

    @Override
    public void update() {
        if (!superstructure.hasGamePiece() && !timerStarted){
            mTimer.reset();
            mTimer.start();
            timerStarted = true;
        }
    }

    @Override
    public void done() {
    }

}
