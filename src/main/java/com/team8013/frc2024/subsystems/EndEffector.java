package com.team8013.frc2024.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team8013.frc2024.Constants;
import com.team8013.frc2024.Ports;
import com.team8013.frc2024.loops.ILooper;
import com.team8013.frc2024.loops.Loop;
import com.team8013.lib.logger.Log;
import com.team8013.lib.requests.Request;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EndEffector extends Subsystem {
    public State mState = State.IDLE;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private boolean hasGamePiece = false;
    private DigitalInput mBeamBreak;

    private final TalonFX mMaster;
    private final TalonFX mSlave;

    // private final SupplyCurrentLimitConfiguration kSupplyCurrentLimit = new
    // SupplyCurrentLimitConfiguration(true, 40, 40,
    // 0.02);

    // private final StatorCurrentLimitConfiguration kDisabledStatorLimit = new
    // StatorCurrentLimitConfiguration(false, 0, 0, 0);
    // private final StatorCurrentLimitConfiguration kEnabledStatorLimit = new
    // StatorCurrentLimitConfiguration(true, 40, 40,
    // 0.0);

    private EndEffector() {
        mMaster = new TalonFX(Ports.END_EFFECTOR_A, Ports.CANBUS);
        mSlave = new TalonFX(Ports.END_EFFECTOR_B, Ports.CANBUS);
        mBeamBreak = new DigitalInput(Ports.END_EFFECTOR_BEAM_BREAK);

        // Customize these configs from constants in the future
        mMaster.getConfigurator().apply(Constants.EndEffectorConstants.endEffectorMotorConfig());
        mSlave.getConfigurator().apply(Constants.EndEffectorConstants.endEffectorMotorConfig());

        mSlave.setControl(new Follower(Ports.END_EFFECTOR_A, false));
        setWantNeutralBrake(false);

    }

    public static EndEffector mInstance;

    public static EndEffector getInstance() {
        if (mInstance == null) {
            mInstance = new EndEffector();
        }
        return mInstance;
    }

    public enum State {
        IDLE(0.0),
        INTAKING(-12.0), // max 12
        OUTTAKING(12.0), // max 12
        SHOOTING(0.0);

        public double voltage;

        State(double voltage) {
            this.voltage = voltage;
        }
    }

    public State getState() {
        return mState;
    }

    public void setState(State state) {
        mState = state;
    }

    public void stop() {
        mState = State.IDLE;
        mPeriodicIO.demand = mState.voltage;
    }

    public void outtake() {
        mState = State.OUTTAKING;
        mPeriodicIO.demand = mState.voltage;
    }

    public void intake() {
        mState = State.INTAKING;
        mPeriodicIO.demand = mState.voltage;
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
        private double position;
        private double velocity;

        private boolean beamBreak;

        // Outputs
        private double demand;
    }

    public void setOpenLoopDemand(double demand) {
        mPeriodicIO.demand = demand;
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
                switch (mState) {
                    case IDLE:
                        mPeriodicIO.demand = mState.voltage;
                        break;
                    case INTAKING:
                        mPeriodicIO.demand = 0.3;
                        break;
                    case OUTTAKING:
                        mPeriodicIO.demand = -0.3; // mState.voltage;
                        break;
                    case SHOOTING:
                        break;
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    @Override
    public void writePeriodicOutputs() {
        if (mState == State.SHOOTING) {
            mMaster.setControl(new MotionMagicVelocityTorqueCurrentFOC(mPeriodicIO.demand));
        } else {
            if (mPeriodicIO.demand > 1 || mPeriodicIO.demand < -1) {
                mMaster.setControl(new VoltageOut(mPeriodicIO.demand));
            } else {
                mMaster.setControl(new DutyCycleOut(mPeriodicIO.demand));
            }
        }

        if (mState == State.INTAKING) {
            if (mPeriodicIO.beamBreak) {
                hasGamePiece = true;
            }
        }
        if (mState == State.OUTTAKING) {
            hasGamePiece = false;
        }
    }

    public Request effectorRequest(State _wantedState) {
        return new Request() {
            @Override
            public void act() {
                setState(_wantedState);
            }

            @Override
            public boolean isFinished() {
                return mPeriodicIO.demand == _wantedState.voltage;
            }
        };
    }

    public Request waitForGamePieceRequest() {
        return new Request() {
            @Override
            public void act() {

            }

            @Override
            public boolean isFinished() {
                return hasGamePiece;
            }
        };
    }

    public void setEndEffectorVelocity(double rpm) {
        if (mState != State.SHOOTING) {
            mState = State.SHOOTING;
        }
        mPeriodicIO.demand = rpm / 60;
    }

    private void setWantNeutralBrake(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        mMaster.setNeutralMode(mode);
        mSlave.setNeutralMode(mode);
    }

    public boolean hasGamePiece() {
        return hasGamePiece;
    }

    @Log
    public double getEndEffectorDemand() {
        return mPeriodicIO.demand;
    }

    @Log
    public double getEndEffectorVoltage() {
        return mPeriodicIO.voltage;
    }

    @Log
    public double getEndEffectorCurrent() {
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
        SmartDashboard.putNumber("Intake Demand", mPeriodicIO.demand);
        SmartDashboard.putNumber("Intake Volts", mPeriodicIO.voltage);
        SmartDashboard.putNumber("Intake Current", mPeriodicIO.current);
        SmartDashboard.putString("Intake State", mState.toString());
        SmartDashboard.putBoolean("Has game piece", hasGamePiece);
        SmartDashboard.putBoolean("END EFFECTOR Beam Break", mPeriodicIO.beamBreak);
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.voltage = mMaster.getMotorVoltage().getValue();
        mPeriodicIO.current = mMaster.getStatorCurrent().getValue();
        mPeriodicIO.position = mMaster.getRotorPosition().getValue();
        mPeriodicIO.velocity = mMaster.getRotorVelocity().getValue();
        mPeriodicIO.beamBreak = mBeamBreak.get();
    }
}