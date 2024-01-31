package com.team8013.frc2024.subsystems;


import com.team8013.frc2024.Constants;
import com.team8013.frc2024.Ports;
import com.team8013.frc2024.loops.ILooper;
import com.team8013.frc2024.loops.Loop;
import com.team8013.lib.Conversions;
import com.team8013.lib.Util;
import com.team8013.lib.logger.Log;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pivot extends Subsystem{

    private static Pivot mInstance;
    private TalonFX mMaster;
    private TalonFX mSlave;
    private CANcoder mCANcoder;

    private mPeriodicIO mPeriodicIO = new mPeriodicIO();
    // private ControlModeState mControlModeState = new ControlModeState();

    // private final StatorCurrentLimitConfiguration kScrapeCurrentLimit = new StatorCurrentLimitConfiguration(true, 2, 2,
    //         0.0);

    public static Pivot getInstance() {
        if (mInstance == null) {
            mInstance = new Pivot();
        }
        return mInstance;
    }

    private Pivot() {
        mMaster = new TalonFX(Ports.PIVOT_A,Ports.CANBUS);
        mSlave = new TalonFX(Ports.PIVOT_B,Ports.CANBUS);
        mCANcoder = new CANcoder(Ports.PIVOT_CANCODER, Ports.CANBUS);
        CANcoderConfiguration CANCoderConfig = Constants.PivotConstants.pivotCancoderConfig();

        //Customize these configs from constants in the future
        mMaster.getConfigurator().apply(Constants.PivotConstants.pivotMotorConfig());
        mSlave.getConfigurator().apply(Constants.PivotConstants.pivotMotorConfig());

        mCANcoder.getConfigurator().apply(CANCoderConfig);

        mSlave.setControl(new Follower(Ports.PIVOT_A, true));
        setWantNeutralBrake(false);
        resetToAbsolute();
    }

    public void resetToAbsolute(){
       double absolutePosition = Conversions.degreesToRotation(getCanCoder().getDegrees(),Constants.PivotConstants.PivotGearRatio);
        mMaster.setPosition(absolutePosition);
    }


    private void setWantNeutralBrake(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        mMaster.setNeutralMode(mode);
        mSlave.setNeutralMode(mode);
    }


    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                resetToAbsolute();
            }

            @Override
            public void onLoop(double timestamp) {

            }

            @Override
            public void onStop(double timestamp) {
                setWantNeutralBrake(true);
            }
        });
    }


    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.mControlModeState == ControlModeState.MOTION_MAGIC){
            mMaster.setControl(new MotionMagicDutyCycle(mPeriodicIO.demand));
        }
        else if (mPeriodicIO.mControlModeState == ControlModeState.OPEN_LOOP){
            if (mPeriodicIO.demand>1||mPeriodicIO.demand<-1){
                mMaster.setControl(new VoltageOut(mPeriodicIO.demand)); //Enable FOC in the future?
            }
            else{
                
                mMaster.setControl(new DutyCycleOut(mPeriodicIO.demand)); //needs a feedforward
            }
        }


    }

    // public Request PivotRequest(double angle, boolean waitForPosition) {
    //     return new Request() {
    //         @Override
    //         public void act() {
    //             setSetpointMotionMagic(angle);
    //             is_climb = false;
    //             is_scraping = false;
    //             updateCurrentLimits();
    //         }

    //         @Override
    //         public boolean isFinished() {
    //             return waitForPosition ? Util.epsilonEquals(mPeriodicIO.position, angle, 3.0) : true;
    //         }
    //     };
    // }

    // public Request climbRequest(double angle) {
    //     return new Request() {
    //         @Override
    //         public void act() {
    //             setSetpointMotionMagic(angle);
    //             is_climb = true;
    //             is_scraping = false;
    //             updateCurrentLimits();
    //         }

    //         @Override
    //         public boolean isFinished() {
    //             return Util.epsilonEquals(mPeriodicIO.position_units, angle, 1.5);
    //         }
    //     };
    // }

    // public Request scrapeRequest(double angle) {
    //     return new SequentialRequest(
    //         scrapeDropRequest(angle),
    //         scrapeHoldRequest(angle)
    //     );
    // }
    

    // private Request scrapeDropRequest(double angle) {
    //     return new Request() {
    //         @Override
    //         public void act() {
    //             setSetpointMotionMagic(angle);
    //             is_scraping = false;
    //             updateCurrentLimits();
    //         }

    //         @Override
    //         public boolean isFinished() {
    //             return Util.epsilonEquals(mPeriodicIO.position_units, angle, 1.5);
    //         }
    //     };
    // }

    // private Request scrapeHoldRequest(double angle) {
    //     return new Request() {
    //         @Override
    //         public void act() {
    //             is_scraping = true;
    //             updateCurrentLimits();
    //         }

    //         @Override
    //         public boolean isFinished() {
    //             return Util.epsilonEquals(mPeriodicIO.position_units, angle, 1.5);
    //         }
    //     };
    // }

    // public Request PivotWaitRequest(double angle) {
    //     return new Request() {
    //         @Override 
    //         public void act() {

    //         }

    //         @Override 
    //         public boolean isFinished() {
    //             return Util.epsilonEquals(mPeriodicIO.position_degrees, angle, 1.0);
    //         }
    //     };
    // }


    public void setSetpointMotionMagic(double degrees) {
                if (mPeriodicIO.mControlModeState != ControlModeState.MOTION_MAGIC) {
                    mPeriodicIO.mControlModeState = ControlModeState.MOTION_MAGIC;
            }
        double rotationDemand = Conversions.degreesToRotation(degrees,Constants.PivotConstants.PivotGearRatio);
        mPeriodicIO.demand = rotationDemand;
    }

    public void setDemandOpenLoop(double demand) {
                if (mPeriodicIO.mControlModeState != ControlModeState.OPEN_LOOP) {
                    mPeriodicIO.mControlModeState = ControlModeState.OPEN_LOOP;
            }
        mPeriodicIO.demand = demand;
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(Util.placeInAppropriate0To360Scope(mCANcoder.getAbsolutePosition().getValueAsDouble()*360 - Constants.PivotConstants.CANCODER_OFFSET,mCANcoder.getAbsolutePosition().getValueAsDouble()*360 - Constants.PivotConstants.CANCODER_OFFSET));
    }

    @Log
    public double getPivotAngleDeg(){
        return mPeriodicIO.position_degrees;
    }

    
    @Log
    public double getPivotDemand(){
        return mPeriodicIO.demand;
    }
    
    @Log
    public double getPivotVelocity(){
        return mPeriodicIO.velocity_radPerSec;
    }
    
    @Log
    public double getPivotVolts(){
        return mPeriodicIO.output_voltage;
    }
    
    @Log
    public double getPivotCurrent(){
        return mPeriodicIO.current;
    }
    

    @Log
    public double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
    
    @Log
    public double getMainMotorBusVolts() {
        return mMaster.getSupplyVoltage().getValueAsDouble();
    }


    public static class mPeriodicIO {
        // Inputs
        public double timestamp = 0.0;
        public double targetVelocity = 0.0;
        public double position_degrees = 0.0;
        public double velocity_radPerSec = 0.0;

        public double current = 0.0;
        public double output_voltage = 0.0;

        // Outputs
        public double demand = 0;
        public ControlModeState mControlModeState = ControlModeState.OPEN_LOOP;
    }

    private enum ControlModeState{
        OPEN_LOOP,
        MOTION_MAGIC
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.position_degrees = Conversions.rotationsToDegrees(mMaster.getRotorPosition().getValueAsDouble(), Constants.PivotConstants.PivotGearRatio);
        mPeriodicIO.current = mMaster.getTorqueCurrent().getValueAsDouble();
        mPeriodicIO.output_voltage = mMaster.getMotorVoltage().getValueAsDouble();
        mPeriodicIO.velocity_radPerSec = Conversions.rotationsToDegrees(mMaster.getVelocity().getValueAsDouble(), Constants.PivotConstants.PivotGearRatio)*Math.PI/180;
    }


    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Pivot Angle (degrees)", mPeriodicIO.position_degrees);
        SmartDashboard.putNumber("Pivot CANCODER (degrees)", getCanCoder().getDegrees());
        SmartDashboard.putNumber("Pivot Motor Rotations", mMaster.getRotorPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Demand", mPeriodicIO.demand);
        SmartDashboard.putNumber("Pivot" + " Velocity rad/s", mPeriodicIO.velocity_radPerSec);
        SmartDashboard.putNumber("Pivot Demand", mPeriodicIO.demand);
        SmartDashboard.putNumber("Pivot Volts", mPeriodicIO.output_voltage);
        SmartDashboard.putNumber("Pivot Current", mPeriodicIO.current);
        SmartDashboard.putString("Pivot Control State", mPeriodicIO.mControlModeState.toString());
    }
}