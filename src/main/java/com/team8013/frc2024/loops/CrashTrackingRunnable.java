package com.team8013.frc2024.loops;

// CREDIT - FRC Team 1323

/**
 * Runnable class with reports all uncaught throws to CrashTracker
 */
public abstract class CrashTrackingRunnable implements Runnable {

    @Override
    public final void run() {
        try {
            runCrashTracked();
        } catch (Throwable t) {
            t.printStackTrace();
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public abstract void runCrashTracked();
}
