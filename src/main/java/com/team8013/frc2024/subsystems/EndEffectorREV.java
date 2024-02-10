package com.team8013.frc2024.subsystems;


import com.team8013.frc2024.Constants;
import com.team8013.frc2024.Ports;
import com.team8013.frc2024.loops.ILooper;
import com.team8013.frc2024.loops.Loop;
import com.team8013.lib.logger.Log;
import com.team8013.lib.requests.Request;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EndEffectorREV extends Subsystem {
    public State mState = State.IDLE;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private boolean hasGamePiece = false;
    private DigitalInput mBeamBreak;

    private final CANSparkFlex mMaster;
    private final CANSparkFlex mSlave;

    // private final SupplyCurrentLimitConfiguration kSupplyCurrentLimit = new
    // SupplyCurrentLimitConfiguration(true, 40, 40,
    // 0.02);

    // private final StatorCurrentLimitConfiguration kDisabledStatorLimit = new
    // StatorCurrentLimitConfiguration(false, 0, 0, 0);
    // private final StatorCurrentLimitConfiguration kEnabledStatorLimit = new
    // StatorCurrentLimitConfiguration(true, 40, 40,
    // 0.0);

    private EndEffectorREV() {
        mMaster = new CANSparkFlex(Ports.END_EFFECTOR_A, MotorType.kBrushless);
        mSlave = new CANSparkFlex(Ports.END_EFFECTOR_B, MotorType.kBrushless);
        //mSlave.follow(mMaster,false); //only duplicates voltage -- not good enough
        mBeamBreak = new DigitalInput(Ports.END_EFFECTOR_BEAM_BREAK);

        mMaster.setInverted(false);
        mSlave.setInverted(false);

        // Customize these configs from constants in the future

        setWantNeutralBrake(false);

    }

    public static EndEffectorREV mInstance;

    public static EndEffectorREV getInstance() {
        if (mInstance == null) {
            mInstance = new EndEffectorREV();
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
            mMaster.set(-0.9);
            mSlave.set(-0.9);
            //mMaster.setControl(new MotionMagicVelocityTorqueCurrentFOC(mPeriodicIO.demand));
        } else {
            if (mPeriodicIO.demand > 1 || mPeriodicIO.demand < -1) {
                mMaster.setVoltage(mPeriodicIO.demand);
                mSlave.setVoltage(mPeriodicIO.demand);
            } else {
                System.out.println("demand: " + mPeriodicIO.demand);
                mMaster.set(mPeriodicIO.demand);
                mSlave.set(mPeriodicIO.demand);
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
        IdleMode mode = brake ? IdleMode.kBrake : IdleMode.kCoast;
        mMaster.setIdleMode(mode);
        mSlave.setIdleMode(mode);
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

    // @Log
    // public double getMainMotorBusVolts() {
    //     return mMaster.getBusVoltage();
    // }

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
        mPeriodicIO.voltage = 0;//mMaster.getBusVoltage();
        mPeriodicIO.current = 0; //mMaster.getOutputCurrent();
        mPeriodicIO.velocity = 0; //mMaster.getAbsoluteEncoder().getVelocity();
        mPeriodicIO.beamBreak = !mBeamBreak.get();
    }
}