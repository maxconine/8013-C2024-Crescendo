package com.team8013.frc2024.subsystems;

import com.team8013.frc2024.Constants;
import com.team8013.frc2024.Ports;
import com.team8013.frc2024.loops.ILooper;
import com.team8013.frc2024.loops.Loop;
import com.team8013.lib.Conversions;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberHook extends Subsystem {

    private static ClimberHook mInstance;
    private TalonFX mMotor;

    private mPeriodicIO mPeriodicIO = new mPeriodicIO();

    public static ClimberHook getInstance() {
        if (mInstance == null) {
            mInstance = new ClimberHook();
        }
        return mInstance;
    }

    private ClimberHook() {
        mMotor = new TalonFX(Ports.CLIMBER_HOOK, Ports.CANBUS_LOWER);
        // Customize these configs from constants in the future
        mMotor.getConfigurator().apply(Constants.ClimberHookConstants.climberHookMotorConfig());

        setWantNeutralBrake(true);
        mMotor.setPosition(0);
    }

    public void resetToAbsolute() {
        // mMotor.setPosition(0);
    }

    public void setWantNeutralBrake(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        mMotor.setNeutralMode(mode);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // resetToAbsolute();
            }

            @Override
            public void onLoop(double timestamp) {
                if (mPeriodicIO.position_degrees<0){
                    mMotor.setPosition(0);
                }

            }

            @Override
            public void onStop(double timestamp) {
                setWantNeutralBrake(true);
            }
        });
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.mControlModeState == ControlModeState.MOTION_MAGIC) {

            mMotor.setControl(new MotionMagicDutyCycle(mPeriodicIO.demand, true, 0, 0, false, false, false));
        } else if (mPeriodicIO.mControlModeState == ControlModeState.OPEN_LOOP) {
            mMotor.setControl(new DutyCycleOut(mPeriodicIO.demand));
        }
    }

    public void setSetpointMotionMagic(double degrees) {
        if (mPeriodicIO.mControlModeState != ControlModeState.MOTION_MAGIC) {
            mPeriodicIO.mControlModeState = ControlModeState.MOTION_MAGIC;
        }
        // if (degrees > Constants.ClimberHookConstants.kMaxAngle){
        // degrees = Constants.ClimberHookConstants.kMaxAngle;
        // }
        // else if (degrees<Constants.ClimberHookConstants.kMinAngle){
        // degrees = Constants.ClimberHookConstants.kMinAngle;
        // }

        double rotationDemand = Conversions.degreesToRotation(degrees, Constants.ClimberHookConstants.kGearRatio);
        mPeriodicIO.demand = rotationDemand;
    }

    public void setDemandOpenLoop(double demand) {
        if (mPeriodicIO.mControlModeState != ControlModeState.OPEN_LOOP) {
            mPeriodicIO.mControlModeState = ControlModeState.OPEN_LOOP;
        }
        mPeriodicIO.demand = demand;
    }

    public double getAngleDeg() {
        return mPeriodicIO.position_degrees;
    }

    public double getTorqueCurrent(){
        return mPeriodicIO.current;
    }

    public static class mPeriodicIO {
        // Inputs
        public double timestamp = 0.0;
        public double targetVelocity = 0.0;
        public double position_degrees = 0.0;
        public double velocity_rps = 0.0;

        public double current = 0.0;
        public double output_voltage = 0.0;

        // Outputs
        public double demand = 0;
        public ControlModeState mControlModeState;
    }

    private enum ControlModeState {
        MOTION_MAGIC,
        OPEN_LOOP
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.position_degrees = Conversions.rotationsToDegrees(mMotor.getRotorPosition().getValue(),
                Constants.ClimberHookConstants.kGearRatio);
        mPeriodicIO.current = mMotor.getTorqueCurrent().getValue();
        mPeriodicIO.output_voltage = mMotor.getMotorVoltage().getValue();
        mPeriodicIO.velocity_rps = Conversions.rotationsToDegrees(mMotor.getVelocity().getValue(),
                Constants.ClimberHookConstants.kGearRatio);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("ClimberHookAngle (degrees)", mPeriodicIO.position_degrees);
        SmartDashboard.putNumber("ClimberHook Motor Rotations", mMotor.getRotorPosition().getValueAsDouble());
        SmartDashboard.putNumber("ClimberHook Demand", mPeriodicIO.demand);
        SmartDashboard.putNumber("ClimberHook Velocity rad/s", mPeriodicIO.velocity_rps);
        SmartDashboard.putNumber("ClimberHook Demand", mPeriodicIO.demand);
        SmartDashboard.putNumber("ClimberHook Volts", mPeriodicIO.output_voltage);
        SmartDashboard.putNumber("ClimberHook Current", mPeriodicIO.current);
        // SmartDashboard.putString("ClimberHook Control State",
        // mPeriodicIO.mControlModeState.toString());
    }
}