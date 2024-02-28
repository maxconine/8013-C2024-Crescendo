package com.team8013.frc2024.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team8013.frc2024.Constants;
import com.team8013.frc2024.Ports;
import com.team8013.frc2024.loops.ILooper;
import com.team8013.frc2024.loops.Loop;
import com.team8013.lib.logger.Log;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private boolean hasGamePiece = false;
    private DigitalInput mBeamBreak;

    private final TalonFX mMaster;
    private final TalonFX mSlave;
    ControlState mControlState;

    // private final SupplyCurrentLimitConfiguration kSupplyCurrentLimit = new
    // SupplyCurrentLimitConfiguration(true, 40, 40,
    // 0.02);

    // private final StatorCurrentLimitConfiguration kDisabledStatorLimit = new
    // StatorCurrentLimitConfiguration(false, 0, 0, 0);
    // private final StatorCurrentLimitConfiguration kEnabledStatorLimit = new
    // StatorCurrentLimitConfiguration(true, 40, 40,
    // 0.0);

    private Shooter() {
        mMaster = new TalonFX(Ports.Shooter_A, Ports.CANBUS_UPPER);
        mSlave = new TalonFX(Ports.Shooter_B, Ports.CANBUS_UPPER);
        mBeamBreak = new DigitalInput(Ports.SHOOTER_BEAM_BREAK);

        // Customize these configs from constants in the future
        mMaster.getConfigurator().apply(Constants.ShooterConstants.shooterMotorConfig());
        mSlave.getConfigurator().apply(Constants.ShooterConstants.shooterMotorConfig());

        mSlave.setControl(new Follower(Ports.Shooter_A, true));
        setWantNeutralBrake(false);
    }

    public static Shooter mInstance;

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    public enum ControlState {
        OPEN_LOOP,
        CLOSED_LOOP,
        SLINGSHOT
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public static class PeriodicIO {
        // Inputs
        private double timestamp;
        private double voltage;
        private double current;
        // private double position;
        private double velocity;

        private boolean beamBreak;

        // Outputs
        private double demand;
    }

    /* SET DEMANDS */

    public void setOpenLoopDemand(double demand) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }
        mPeriodicIO.demand = demand;
    }

    /**
     * rpm
     */
    public void setClosedLoopDemand(double rps) {
        if (mControlState != ControlState.CLOSED_LOOP) {
            mControlState = ControlState.CLOSED_LOOP;
        }
        mPeriodicIO.demand = rps;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                stop();
            }

            @Override
            public void onLoop(double timestamp) {

            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    @Override
    public void writePeriodicOutputs() {
        if (mControlState == ControlState.OPEN_LOOP) {
            if (mPeriodicIO.demand > 1 || mPeriodicIO.demand < -1) {
                mMaster.setControl(new VoltageOut(mPeriodicIO.demand, true, false, false, false));
            } else {
                mMaster.setControl(new DutyCycleOut(mPeriodicIO.demand, true, false, false, false));
            }
        } else if (mControlState == ControlState.CLOSED_LOOP) {
            mMaster.setControl(
                    new MotionMagicVelocityDutyCycle(mPeriodicIO.demand, 0, true, 0, 0, false, false, false));
        }
        // else if (mControlState == ControlState.SLINGSHOT){
        // mMaster.setControl(new TorqueCurrentFOC(780,1,1,false,false,false));
        // }
    }

    // public Request effectorRequest (State _wantedState) {
    // return new Request () {
    // @Override
    // public void act() {
    // setState(_wantedState);
    // }
    // @Override
    // public boolean isFinished() {
    // return mPeriodicIO.demand == _wantedState.voltage;
    // }
    // };
    // }

    // public Request waitForGamePieceRequest () {
    // return new Request() {
    // @Override
    // public void act () {

    // }
    // @Override
    // public boolean isFinished() {
    // return hasGamePiece;
    // }
    // };
    // }

    private void setWantNeutralBrake(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        mMaster.setNeutralMode(mode);
        mSlave.setNeutralMode(mode);
    }

    // public boolean hasGamePiece() {
    // return hasGamePiece;
    // }

    public boolean getBeamBreak() {
        return mPeriodicIO.beamBreak;
    }

    @Log
    public double getShooterDemand() {
        return mPeriodicIO.demand;
    }

    @Log
    public double getShooterVoltage() {
        return mPeriodicIO.voltage;
    }

    @Log
    public double getShooterCurrent() {
        return mPeriodicIO.current;
    }

    @Log
    public double getTimestamp() {
        return mPeriodicIO.timestamp;
    }

    @Log
    public double getVelocity() {
        return mPeriodicIO.velocity;
    }

    @Log
    public double getMainMotorBusVolts() {
        return mMaster.getSupplyVoltage().getValue();
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Shooter Demand", mPeriodicIO.demand);
        SmartDashboard.putNumber("Shooter Volts", mPeriodicIO.voltage);
        SmartDashboard.putNumber("Shooter Current", mPeriodicIO.current);
        SmartDashboard.putBoolean("Has game piece", hasGamePiece);
        SmartDashboard.putBoolean("Shooter Beam Break", mPeriodicIO.beamBreak);
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.voltage = mMaster.getMotorVoltage().getValue();
        mPeriodicIO.current = mMaster.getStatorCurrent().getValue();
        // mPeriodicIO.position = mMaster.getRotorPosition().getValue();
        mPeriodicIO.velocity = mMaster.getRotorVelocity().getValue();
        mPeriodicIO.beamBreak = !mBeamBreak.get();
    }
}