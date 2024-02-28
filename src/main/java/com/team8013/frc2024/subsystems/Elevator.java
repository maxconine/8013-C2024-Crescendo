package com.team8013.frc2024.subsystems;

import com.team8013.frc2024.Constants;
import com.team8013.frc2024.Ports;
import com.team8013.frc2024.loops.ILooper;
import com.team8013.frc2024.loops.Loop;
import com.team8013.lib.Conversions;
import com.team8013.lib.logger.Log;
import com.team8013.lib.requests.Request;
import com.team8013.lib.util.DelayedBoolean;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends Subsystem {

    PeriodicIO mPeriodicIO = new PeriodicIO();

    public static Elevator mInstance;
    private final TalonFX mMaster;
    private final TalonFX mSlave;

    public static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator();
        }
        return mInstance;
    }

    private Elevator() {
        mMaster = new TalonFX(Ports.ELEVATOR_A, Ports.CANBUS_UPPER);
        mSlave = new TalonFX(Ports.ELEVATOR_B, Ports.CANBUS_UPPER);

        // Customize these configs from constants in the future
        mMaster.getConfigurator().apply(Constants.ElevatorConstants.elevatorFastMotorConfig());
        mSlave.getConfigurator().apply(Constants.ElevatorConstants.elevatorFastMotorConfig());

        mSlave.setControl(new Follower(Ports.ELEVATOR_A, true));
        setNeutralBrake(false);
    }

    // Homing refers to moving the elevator into it's "zero" position. Needs to be
    // done at the start of every match
    private boolean mHoming = true;
    private boolean mNeedsToHome = false;
    private final DelayedBoolean mHomingDelay = new DelayedBoolean(Timer.getFPGATimestamp(), 0.2);

    public void setNeutralBrake(boolean brake) {
        NeutralModeValue wantedMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        mMaster.setNeutralMode(wantedMode);
        mSlave.setNeutralMode(wantedMode);
    }

    public void setWantHome(boolean home) {
        mHoming = home;
        // once homing is started, no longer needs to home
        if (mHoming) {
            mNeedsToHome = false;
        }
        // force update state
        writePeriodicOutputs();
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledILooper) {
        mEnabledILooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setNeutralBrake(true);
                // in the future we might want to do this
                // setSetpointMotionMagic(0.0);
            }

            @Override
            public void onLoop(double timestamp) {
                // constantly re-homing unless in open loop, so if it is disabled we can push it
                // in and it will remember the most "in" spot
                if (atHomingLocation() && mNeedsToHome) {
                    setWantHome(true);
                } else if (mPeriodicIO.mControlModeState != ControlModeState.OPEN_LOOP) {
                    setWantHome(false);
                }
            }

            @Override
            public void onStop(double timestamp) {
                setNeutralBrake(true);
            }
        });
    }

    public Request elevatorRequest(double length, boolean waitForPosition) {
        return new Request() {

            @Override
            public void act() {
                setSetpointMotionMagic(length);
            }

            @Override
            public boolean isFinished() {
                return !waitForPosition || Util.epsilonEquals(mPeriodicIO.position, length, 0.1);
            }

        };
    }

    public Request elevatorWaitRequest(double length) {
        return new Request() {
            @Override
            public void act() {

            }

            @Override
            public boolean isFinished() {
                return Util.epsilonEquals(mPeriodicIO.position, length, 0.2);
            }
        };
    }

    public Request elevatorTuckWaitRequest(double length) {
        return new Request() {
            @Override
            public void act() {

            }

            @Override
            public boolean isFinished() {
                return mPeriodicIO.position < length;
            }
        };
    }

    public Request elevatorExtendWaitRequest(double length) {
        return new Request() {
            @Override
            public void act() {

            }

            @Override
            public boolean isFinished() {
                return mPeriodicIO.position > length;
            }
        };
    }

    /**
     * 
     * @param units meters to extend elevator to
     */
    public void setSetpointMotionMagic(double distance) {
        // if (distance != 0.0) { //why?
        // mNeedsToHome = true;
        // }
        if (mPeriodicIO.mControlModeState != ControlModeState.MOTION_MAGIC) {
            mPeriodicIO.mControlModeState = ControlModeState.MOTION_MAGIC;
        }
        double rotationDemand = Conversions.metersToRotations(distance, Constants.ElevatorConstants.kWheelCircumference,
                Constants.ElevatorConstants.kGearRatio);
        mPeriodicIO.demand = rotationDemand;
    }

    /**
     * 
     * @param demand if >1 controls voltage, if <1 controls percent output
     */
    public void setOpenLoopDemand(double demand) {
        if (mPeriodicIO.mControlModeState != ControlModeState.OPEN_LOOP) {
            mPeriodicIO.mControlModeState = ControlModeState.OPEN_LOOP;
        }
        mPeriodicIO.demand = demand;
    }

    @Override
    public synchronized void writePeriodicOutputs() {

        if ((mPeriodicIO.torqueCurrent < -20) && mPeriodicIO.velocity < 0.1 && mPeriodicIO.position < 0) {
            mHoming = false;
            zeroSensors();
            setSetpointMotionMagic(0.01);
        }

        if (mHoming) { // sets it moving backward until velocity slows down
            mMaster.setControl(new VoltageOut(-3));
            if (mHomingDelay.update(Timer.getFPGATimestamp(),
                    Util.epsilonEquals(mPeriodicIO.velocity, 0.0, 0.1))) { // is this motor velocity or elevator
                                                                           // velocity
                zeroSensors();
                setSetpointMotionMagic(0.01);
                mHoming = false;
            }
        } else if (mPeriodicIO.mControlModeState == ControlModeState.OPEN_LOOP) {
            if (mPeriodicIO.demand > 1 || mPeriodicIO.demand < -1) {
                mMaster.setControl(new VoltageOut(mPeriodicIO.demand)); // Enable FOC in the future?
            } else {
                mMaster.setControl(new DutyCycleOut(mPeriodicIO.demand));
            }

        } else if (mPeriodicIO.mControlModeState == ControlModeState.MOTION_MAGIC) {
            mMaster.setControl(new MotionMagicDutyCycle(mPeriodicIO.demand, true, 0, 0, false, false, false));
        }
    }

    public void zeroSensors() {
        mMaster.setPosition(0);
    }

    public void setDemandOpenLoop(double demand) {
        if (mPeriodicIO.mControlModeState != ControlModeState.OPEN_LOOP) {
            mPeriodicIO.mControlModeState = ControlModeState.OPEN_LOOP;
        }
        mPeriodicIO.demand = demand;
    }

    public boolean atHomingLocation() {
        return mPeriodicIO.position < 0.0
                || Util.epsilonEquals(mPeriodicIO.position, 0.0, 0.05);
    }

    public void setMotorConfig(TalonFXConfiguration config) {
        mMaster.getConfigurator().apply(config);
    }

    @Log
    public double getElevatorUnits() {
        return mPeriodicIO.position;
    }

    @Log
    public double getElevatorDemand() {
        return mPeriodicIO.demand;
    }

    @Log
    public double getElevatorVelocity() {
        return mPeriodicIO.velocity;
    }

    @Log
    public double getElevatorVolts() {
        return mPeriodicIO.voltage;
    }

    @Log
    public double getElevatorCurrent() {
        return mPeriodicIO.current;
    }

    @Log
    public boolean getElevatorHoming() {
        return mHoming;
    }

    @Log
    public double getTimestamp() {
        return mPeriodicIO.timestamp;
    }

    public static class PeriodicIO {
        // Inputs
        private double timestamp;
        private double voltage;
        private double current;
        private double position;
        private double velocity;
        private double torqueCurrent;

        private ControlModeState mControlModeState;

        // Outputs
        private double demand;
    }

    private enum ControlModeState {
        OPEN_LOOP,
        MOTION_MAGIC
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Elevator Position Meters", mPeriodicIO.position);
        SmartDashboard.putNumber("Elevator Position Inches", Conversions.metersToInches(mPeriodicIO.position));
        SmartDashboard.putNumber("Elevator Motor Rotations", mMaster.getRotorPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Demand", mPeriodicIO.demand);
        SmartDashboard.putNumber("Elevator Velocity", mPeriodicIO.velocity);
        SmartDashboard.putNumber("Elevator Output Volts", mPeriodicIO.voltage);
        SmartDashboard.putNumber("Elevator Current", mPeriodicIO.current);
        SmartDashboard.putNumber("Elevator Torque Current", mPeriodicIO.torqueCurrent);
        SmartDashboard.putBoolean("Elevator Homing", mHoming);
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.voltage = mMaster.getMotorVoltage().getValue();
        mPeriodicIO.current = mMaster.getStatorCurrent().getValue();
        mPeriodicIO.position = Conversions.rotationsToMeters(mMaster.getRotorPosition().getValue(),
                Constants.ElevatorConstants.kWheelCircumference, Constants.ElevatorConstants.kGearRatio);
        mPeriodicIO.velocity = mMaster.getRotorVelocity().getValue();
        mPeriodicIO.torqueCurrent = mMaster.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public void stop() {
        super.stop();
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

}