package com.team8013.frc2024.subsystems;

import com.team8013.frc2024.Constants;
import com.team8013.frc2024.Ports;
import com.team8013.frc2024.loops.ILooper;
import com.team8013.frc2024.loops.Loop;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EndEffectorREV extends Subsystem {
    public State mState = State.IDLE;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private DigitalInput mBeamBreak;

    private RelativeEncoder m_encoderMaster;
    private RelativeEncoder m_encoderSlave;

    private final CANSparkFlex mMaster;
    private final CANSparkFlex mSlave;

    private PIDController pidMaster;
    private PIDController pidSlave;

    private EndEffectorREV() {
        mMaster = new CANSparkFlex(Ports.END_EFFECTOR_A, MotorType.kBrushless);
        mSlave = new CANSparkFlex(Ports.END_EFFECTOR_B, MotorType.kBrushless);
        mBeamBreak = new DigitalInput(Ports.END_EFFECTOR_BEAM_BREAK);

        mMaster.setInverted(false);
        mSlave.setInverted(false);

        mMaster.setIdleMode(IdleMode.kBrake);
        mSlave.setIdleMode(IdleMode.kBrake);

        // Customize these configs from constants in the future

        pidMaster = new PIDController(Constants.EndEffectorConstants.kP, Constants.EndEffectorConstants.kI,
                Constants.EndEffectorConstants.kD);
        pidSlave = new PIDController(Constants.EndEffectorConstants.kPSlave, Constants.EndEffectorConstants.kI,
                Constants.EndEffectorConstants.kD);

        m_encoderMaster = mMaster.getEncoder();
        m_encoderSlave = mSlave.getEncoder();

    }

    public static EndEffectorREV mInstance;

    public static EndEffectorREV getInstance() {
        if (mInstance == null) {
            mInstance = new EndEffectorREV();
        }
        return mInstance;
    }

    public enum State {
        IDLE,
        INTAKING,
        OUTTAKING,
        CLOSED_LOOP,
        OPEN_LOOP
    }

    public State getState() {
        return mState;
    }

    public void setState(State state) {
        if (mState != state) {
            mState = state;
        }
        mState = state;
    }

    public void stop() {
        mState = State.IDLE;
        mPeriodicIO.demandMaster = 0;
        mPeriodicIO.demandSlave = 0;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public static class PeriodicIO {
        // Inputs
        private double voltage;
        private double current;
        private double velocityMaster;
        private double velocitySlave;

        private boolean beamBreak;

        // Outputs
        private double demandMaster;
        private double demandSlave;
    }

    public void setOpenLoopDemand(double demand) {
        if (mState != State.OPEN_LOOP) {
            mState = State.OPEN_LOOP;
        }
        mPeriodicIO.demandMaster = demand;
        mPeriodicIO.demandSlave = demand;
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
                        mPeriodicIO.demandMaster = 0;
                        mPeriodicIO.demandSlave = 0;
                        break;
                    case CLOSED_LOOP:
                        mPeriodicIO.demandMaster = pidMaster.calculate(mPeriodicIO.velocityMaster,
                                mPeriodicIO.demandMaster);
                        mPeriodicIO.demandSlave = pidSlave.calculate(mPeriodicIO.velocitySlave,
                                mPeriodicIO.demandSlave);
                        break;
                    case OPEN_LOOP:
                        break;
                    case INTAKING:
                        mPeriodicIO.demandMaster = 0.35;
                        mPeriodicIO.demandSlave = 0.35;
                        break;
                    case OUTTAKING:
                        mPeriodicIO.demandMaster = -0.3;
                        mPeriodicIO.demandSlave = -0.3;
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
        mMaster.set(mPeriodicIO.demandMaster);
        mSlave.set(mPeriodicIO.demandSlave);
    }

    public void setEndEffectorClosedLoop(double rpmMaster, double rpmSlave) {
        if (mState != State.CLOSED_LOOP) {
            mState = State.CLOSED_LOOP;
        }
        mPeriodicIO.demandMaster = rpmMaster * 2.5;
        mPeriodicIO.demandSlave = rpmSlave * 2.5;

    }

    public boolean hasGamePiece() {
        return mPeriodicIO.beamBreak;
    }

    public double getEndEffectorMasterDemand() {
        return mPeriodicIO.demandMaster;
    }

    public double getEndEffectorVoltage() {
        return mPeriodicIO.voltage;
    }

    public double getEndEffectorCurrent() {
        return mPeriodicIO.current;
    }

    public double getVelocity() {
        return mPeriodicIO.velocityMaster;
    }

    // @Log
    // public double getMainMotorBusVolts() {
    // return mMaster.getBusVoltage();
    // }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("END EFFECTOR Demand Master", mPeriodicIO.demandMaster);
        SmartDashboard.putNumber("Intake Volts", mPeriodicIO.voltage);
        SmartDashboard.putNumber("Intake Current", mPeriodicIO.current);
        SmartDashboard.putNumber("END EFFECTOR Master Velocity", mPeriodicIO.velocityMaster);
        SmartDashboard.putNumber("END EFFECTOR SLAVE VELOCITY", mPeriodicIO.velocitySlave);
        SmartDashboard.putString("Intake State", mState.toString());
        SmartDashboard.putBoolean("END EFFECTOR Beam Break", mPeriodicIO.beamBreak);
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.voltage = mMaster.getBusVoltage();
        mPeriodicIO.current = mMaster.getOutputCurrent();
        mPeriodicIO.velocityMaster = m_encoderMaster.getVelocity();
        mPeriodicIO.velocitySlave = m_encoderSlave.getVelocity();
        mPeriodicIO.beamBreak = !mBeamBreak.get();
    }
}