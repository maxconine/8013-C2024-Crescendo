package com.team8013.frc2024.auto.actions;

import edu.wpi.first.wpilibj.Timer;

/**
 * Action to wait for a given amount of time To use this Action, call runAction(new WaitAction(your_time))
 */
public class NewWaitAction implements Action {

    private double mTimeToWait;
    private Timer mTimer;

    public NewWaitAction(double timeToWait) {
        mTimeToWait = timeToWait;
    }

    @Override
    public boolean isFinished() {
        return mTimer.get() >= mTimeToWait;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mTimer.stop();
        mTimer.reset();
        mTimer.start();
    }
}
