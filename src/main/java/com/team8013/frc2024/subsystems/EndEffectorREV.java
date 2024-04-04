package com.team8013.frc2024.subsystems;

import com.team8013.frc2024.Constants;
import com.team8013.frc2024.Ports;
import com.team8013.frc2024.loops.ILooper;
// import revrobotics.RelativeEncoder;
import com.team8013.frc2024.loops.Loop;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EndEffectorREV extends Subsystem {
    private static EndEffectorREV mInstance;
    public State mState = State.IDLE;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private DigitalInput mBeamBreak;

    private RelativeEncoder mEncoderTop;
    private RelativeEncoder mEncoderBottom;

    private CANSparkFlex mTopMotor;
    private CANSparkFlex mBottomMotor;

    private SparkPIDController pidTopRoller;
    private SparkPIDController pidBottomRoller;

    //private double kPSubWof, kPPodium, kI, kD, kIz, kFFMasterSubWof, kFFSlaveSubWof, kFFMasterPodium, kFFSlavePodium, kMaxOutput, kMinOutput;
    private int slotID;

    private EndEffectorREV(){
        mTopMotor = new CANSparkFlex(Ports.END_EFFECTOR_A, MotorType.kBrushless); //top
        mBottomMotor = new CANSparkFlex(Ports.END_EFFECTOR_B, MotorType.kBrushless); //bottom
        mBeamBreak = new DigitalInput(Ports.END_EFFECTOR_BEAM_BREAK);

        mTopMotor.clearFaults();
        mBottomMotor.clearFaults();

        //doesn't seem to do anything
        // mTopMotor.enableVoltageCompensation(12);
        // mBottomMotor.enableVoltageCompensation(12);

        mBottomMotor.setIdleMode(IdleMode.kCoast);
        mTopMotor.setIdleMode(IdleMode.kBrake);

        // mTopMotor.setInverted(false);
        // mBottomMotor.setInverted(false);

        // Customize these configs from constants in the future

        pidTopRoller = mTopMotor.getPIDController();
        pidBottomRoller = mBottomMotor.getPIDController();

        // pidTopRoller.setIZone(0.01);
        // pidBottomRoller.setIZone(0.01);

        // pidTopRoller = new PIDController(Constants.EndEffectorConstants.kP,
        // Constants.EndEffectorConstants.kI,
        // Constants.EndEffectorConstants.kD);
        // pidBottomRoller = new PIDController(Constants.EndEffectorConstants.kPSlave,
        // Constants.EndEffectorConstants.kI,
        // Constants.EndEffectorConstants.kD);

        mEncoderTop = mTopMotor.getEncoder();
        mEncoderBottom = mBottomMotor.getEncoder();

        /*
         * Slot 1: Subwoofer (4500 RPM)
         * 
         * 
         * Slot 2: Podium (5700)
         * TODO:
         * Set all values to 0 and tune Feed Forward to hit ~4700 rpm (over max
         * velocity)
         * Next, tune the P starting at 0.00002 until target velocity is aquired
         * Once you get a small bit of oscilation, lower P a little and set I to very
         * low (0.00000000015)
         * 
         * Another method:
         * fine tune rpm values, wait for the PID loop to either overshoot or to be
         * within a certain error range to switch to this
         * spool up pid values for 5700 rpm for a podium shot
         */


        // set PID coefficients
        pidTopRoller.setP(Constants.EndEffectorConstants.kPSubWof,0);
        pidTopRoller.setP(Constants.EndEffectorConstants.kPFast,1);
        pidTopRoller.setI(0);
        pidTopRoller.setD(0);
        pidTopRoller.setIZone(0);
        pidTopRoller.setFF(Constants.EndEffectorConstants.kFFTopSubwoofer,0);
        pidTopRoller.setFF(Constants.EndEffectorConstants.kFFTopFast,1);
        pidTopRoller.setOutputRange(Constants.EndEffectorConstants.kMinOutput, Constants.EndEffectorConstants.kMaxOutput);

        pidBottomRoller.setP(Constants.EndEffectorConstants.kPSubWof,0);
        pidTopRoller.setP(Constants.EndEffectorConstants.kPFast,1);
        pidBottomRoller.setI(0);
        pidBottomRoller.setD(0);
        pidBottomRoller.setIZone(0);
        pidBottomRoller.setFF(Constants.EndEffectorConstants.kFFBottomSubwoofer,0);
        pidBottomRoller.setFF(Constants.EndEffectorConstants.kFFBottomFast, 1);
        pidBottomRoller.setOutputRange(Constants.EndEffectorConstants.kMinOutput, Constants.EndEffectorConstants.kMaxOutput);

    }

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

    public void setOpenLoopDemand(double demandMaster, double demandSlave) {
        if (mState != State.OPEN_LOOP) {
            mState = State.OPEN_LOOP;
        }
        mPeriodicIO.demandMaster = demandMaster;
        mPeriodicIO.demandSlave = demandSlave;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {

            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }

    @Override
    public void writePeriodicOutputs() {

        if (mState == State.CLOSED_LOOP) {
            pidTopRoller.setReference(mPeriodicIO.demandMaster, CANSparkFlex.ControlType.kVelocity,slotID);
            pidBottomRoller.setReference(mPeriodicIO.demandSlave, CANSparkFlex.ControlType.kVelocity,slotID);
            // mPeriodicIO.demandMaster = pidTopRoller.calculate(mPeriodicIO.velocityMaster,
            // mPeriodicIO.demandMaster);
            // mPeriodicIO.demandSlave = pidBottomRoller.calculate(mPeriodicIO.velocitySlave,
            // mPeriodicIO.demandSlave);
            SmartDashboard.putString("END EFFECTOR STATE", "CLOSED LOOP");
        } else {

            if (mState == State.IDLE) {
                mPeriodicIO.demandMaster = 0;
                mPeriodicIO.demandSlave = 0;
                SmartDashboard.putString("END EFFECTOR STATE", "IDLE");
            } else if (mState == State.INTAKING) {
                mPeriodicIO.demandMaster = Constants.EndEffectorConstants.kGroundIntakeDemand; // 0.605 //.68 BEFORE WPI
                mPeriodicIO.demandSlave = Constants.EndEffectorConstants.kGroundIntakeDemand; // 0.615
                SmartDashboard.putString("END EFFECTOR STATE", "INTAKING");
            } else if (mState == State.OUTTAKING) {
                mPeriodicIO.demandMaster = Constants.EndEffectorConstants.kOuttakingDemandTop; // was 0.35
                mPeriodicIO.demandSlave = Constants.EndEffectorConstants.kOuttakingDemandBottom; // was 0.35
                SmartDashboard.putString("END EFFECTOR STATE", "OUTTAKING");
            } else if (mState == State.OPEN_LOOP) {
                SmartDashboard.putString("END EFFECTOR STATE", "OPEN LOOP");
            }

            mTopMotor.set(mPeriodicIO.demandMaster);
            mBottomMotor.set(mPeriodicIO.demandSlave);
        }
    }

    public void setEndEffectorClosedLoop(double rpmTopMotor, double rpmBottomMotor) {
        if (mState != State.CLOSED_LOOP) {
            mState = State.CLOSED_LOOP;
        }
        
        //set slot id
        if (rpmTopMotor <5500){
            slotID = 0;
        }
        else {
            slotID = 1;
        }
        mPeriodicIO.demandMaster = rpmTopMotor;
        mPeriodicIO.demandSlave = rpmBottomMotor;

    }

    public void setEndEffectorClosedLoop(double rpmBoth){
        setEndEffectorClosedLoop(rpmBoth, rpmBoth);
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

    public double getVelocityMaster() {
        return mPeriodicIO.velocityMaster;
    }

    public double getVelocitySlave(){
        return mPeriodicIO.velocitySlave;
    }

    // @Log
    // public double getMainMotorBusVolts() {
    // return mTopMotor.getBusVoltage();
    // }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("END EFFECTOR Demand Master", mPeriodicIO.demandMaster);
        SmartDashboard.putNumber("Intake Volts", mPeriodicIO.voltage);
        SmartDashboard.putNumber("Intake Current", mPeriodicIO.current);
        SmartDashboard.putNumber("END EFFECTOR Master Velocity", mPeriodicIO.velocityMaster);
        SmartDashboard.putNumber("END EFFECTOR SLAVE VELOCITY", mPeriodicIO.velocitySlave);
        SmartDashboard.putBoolean("END EFFECTOR Beam Break", mPeriodicIO.beamBreak);
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.voltage = mTopMotor.getBusVoltage();
        mPeriodicIO.current = mTopMotor.getOutputCurrent();
        mPeriodicIO.velocityMaster = mEncoderTop.getVelocity();
        mPeriodicIO.velocitySlave = mEncoderBottom.getVelocity();
        mPeriodicIO.beamBreak = !mBeamBreak.get();
    }
}