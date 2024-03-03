package com.team8013.frc2024.subsystems;

import com.team8013.frc2024.Ports;
import com.team8013.frc2024.loops.ILooper;
import com.team8013.frc2024.loops.Loop;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EndEffectorREV extends Subsystem {
    private static EndEffectorREV mInstance;
    public State mState = State.IDLE;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private DigitalInput mBeamBreak;

    private RelativeEncoder m_encoderMaster;
    private RelativeEncoder m_encoderSlave;

    private CANSparkFlex mMaster;
    private CANSparkFlex mSlave;

    private SparkPIDController pidMaster;
    private SparkPIDController pidSlave;

    private double kP, kI, kD, kIz, kFFMaster, kFFSlave, kMaxOutput, kMinOutput;

    private EndEffectorREV() {
        mMaster = new CANSparkFlex(Ports.END_EFFECTOR_A, MotorType.kBrushless);
        mSlave = new CANSparkFlex(Ports.END_EFFECTOR_B, MotorType.kBrushless);
        mBeamBreak = new DigitalInput(Ports.END_EFFECTOR_BEAM_BREAK);

        mMaster.clearFaults();
        mSlave.clearFaults();

        // mMaster.enableVoltageCompensation(12);
        // mSlave.enableVoltageCompensation(12);

        mSlave.setIdleMode(IdleMode.kCoast);



        // mMaster.setInverted(false);
        // mSlave.setInverted(false);

        // Customize these configs from constants in the future

        pidMaster = mMaster.getPIDController();
        pidSlave = mSlave.getPIDController();

        // pidMaster.setIZone(0.01);
        // pidSlave.setIZone(0.01);

        // pidMaster = new PIDController(Constants.EndEffectorConstants.kP, Constants.EndEffectorConstants.kI,
        //         Constants.EndEffectorConstants.kD);
        // pidSlave = new PIDController(Constants.EndEffectorConstants.kPSlave, Constants.EndEffectorConstants.kI,
        //         Constants.EndEffectorConstants.kD);

        m_encoderMaster = mMaster.getEncoder();
        m_encoderSlave = mSlave.getEncoder();

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

        // PID coefficients for fine targeting
        kP = 0.00022;
        kI = 0;
        kD = 0;
        kIz = 0.00;
        kFFMaster = 0.00015;
        kFFSlave = 0.000148;
        kMaxOutput = 1;
        kMinOutput = -1;

        // set PID coefficients
        pidMaster.setP(kP);
        pidMaster.setI(kI);
        pidMaster.setD(kD);
        pidMaster.setIZone(kIz);
        pidMaster.setFF(kFFMaster);
        pidMaster.setOutputRange(kMinOutput, kMaxOutput);

        pidSlave.setP(kP);
        pidSlave.setI(kI);
        pidSlave.setD(kD);
        pidSlave.setIZone(kIz);
        pidSlave.setFF(kFFSlave);
        pidSlave.setOutputRange(kMinOutput,kMaxOutput);

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
    public void writePeriodicOutputs(){

        if (mState == State.CLOSED_LOOP) {
            pidMaster.setReference(mPeriodicIO.demandMaster, CANSparkFlex.ControlType.kVelocity);
            pidSlave.setReference(mPeriodicIO.demandSlave, CANSparkFlex.ControlType.kVelocity);
            // mPeriodicIO.demandMaster = pidMaster.calculate(mPeriodicIO.velocityMaster,
            //         mPeriodicIO.demandMaster);
            // mPeriodicIO.demandSlave = pidSlave.calculate(mPeriodicIO.velocitySlave,
            //         mPeriodicIO.demandSlave);
            SmartDashboard.putString("END EFFECTOR STATE", "CLOSED LOOP");
        }
        else{

        if (mState == State.IDLE) {
            mPeriodicIO.demandMaster = 0;
            mPeriodicIO.demandSlave = 0;
            SmartDashboard.putString("END EFFECTOR STATE", "IDLE");
        } 
        else if (mState == State.INTAKING) {
            mPeriodicIO.demandMaster = 0.705; //0.605
            mPeriodicIO.demandSlave = 0.715; //0.615
            SmartDashboard.putString("END EFFECTOR STATE", "INTAKING");
        } else if (mState == State.OUTTAKING) {
            mPeriodicIO.demandMaster = -0.50; //was 0.35
            mPeriodicIO.demandSlave = -0.55; //was 0.35
            SmartDashboard.putString("END EFFECTOR STATE", "OUTTAKING");
        } else if (mState == State.OPEN_LOOP) {
            SmartDashboard.putString("END EFFECTOR STATE", "OPEN LOOP");
        }

        mMaster.set(mPeriodicIO.demandMaster);
        mSlave.set(mPeriodicIO.demandSlave);
    }
    }

    public void setEndEffectorClosedLoop(double rpmMaster, double rpmSlave) {
        if (mState != State.CLOSED_LOOP) {
            mState = State.CLOSED_LOOP;
        }
        mPeriodicIO.demandMaster = rpmMaster;
        mPeriodicIO.demandSlave = rpmSlave;

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