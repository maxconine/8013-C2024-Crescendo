package com.team8013.frc2024.auto.actions;

import com.team8013.frc2024.subsystems.EndEffectorREV;

public class WaitForAquisition implements Action {
    EndEffectorREV effector;

    public WaitForAquisition() {
        effector = EndEffectorREV.getInstance();
    }

    @Override
    public boolean isFinished() {
        return effector.hasGamePiece();
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
