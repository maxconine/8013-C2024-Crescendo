package com.team8013.frc2024.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

    // private final SupplyCurrentLimitConfiguration kSupplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 40,
    //         0.02);
   
    // private final StatorCurrentLimitConfiguration kDisabledStatorLimit = new StatorCurrentLimitConfiguration(false, 0, 0, 0);
    // private final StatorCurrentLimitConfiguration kEnabledStatorLimit = new StatorCurrentLimitConfiguration(true, 40, 40,
    //         0.0);
        
    private EndEffector() {
        mMaster = new TalonFX(Ports.END_EFFECTOR_A, Ports.CANBUS);
        mSlave = new TalonFX(Ports.END_EFFECTOR_B, Ports.CANBUS);
        mBeamBreak = new DigitalInput(Ports.END_EFFECTOR_BEAM_BREAK);

        //Customize these configs from constants in the future
        mMaster.getConfigurator().apply(new TalonFXConfiguration());
        mSlave.getConfigurator().apply(new TalonFXConfiguration());

        mSlave.setControl(new Follower(Ports.END_EFFECTOR_A, true));
        setWantNeutralBrake(false);

        // mMaster.config_kP(0, 1.5, Constants.kLongCANTimeoutMs);
        // mMaster.config_kI(0, 0.0, Constants.kLongCANTimeoutMs);
        // mMaster.config_kD(0, 0.5, Constants.kLongCANTimeoutMs);
        // mMaster.config_kF(0, 0.0, Constants.kLongCANTimeoutMs);
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
        INTAKING(-8.0), //max 12
        OUTTAKING(8.0); //max 12
        
        public double voltage;
        State (double voltage) {
            this.voltage = voltage;
        }
    }

    public State getState() {
        return mState;
    }

    /**
     * 
     * @param state the state to set the intake to, Idle, Intaking, or Outtaking
     */
    public void setState(State state) {
        if (mState != state) {
            if (state != State.IDLE) {
                hasGamePiece = false;
            }
            //could change current configs depending
            // if (state == State.INTAKING) {
            //     mMaster.configStatorCurrentLimit(kEnabledStatorLimit, Constants.kCANTimeoutMs);
            // } else {
            //     mMaster.configStatorCurrentLimit(kDisabledStatorLimit, Constants.kCANTimeoutMs);
            // }
        }
        mState = state;
    }

    @Override
    public void stop() {
        mPeriodicIO.demand = 0.0;
        setState(State.IDLE);
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
                setState(State.IDLE);
            }

            @Override
            public void onLoop(double timestamp) {
                switch (mState) {
                    case IDLE:
                        mPeriodicIO.demand = mState.voltage; 
                        break; 
                    case INTAKING:
                        mPeriodicIO.demand = mState.voltage;

                        if (mPeriodicIO.beamBreak == true) {
                            hasGamePiece = true; 
                        } 

                        break; 
                    case OUTTAKING:
                        mPeriodicIO.demand = mState.voltage;
                        hasGamePiece = false;
                        break;
                }
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }
    @Override
    public void writePeriodicOutputs() {       
        if (mPeriodicIO.demand>1||mPeriodicIO.demand<-1){     
            //mMaster.setControl(new VoltageOut(mPeriodicIO.demand)); //could enable FOC in the future
        }
        else{
            //mMaster.setControl(new DutyCycleOut(mPeriodicIO.demand));
        }
    }

    public Request effectorRequest (State _wantedState) {
        return new Request () {
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

    public Request waitForGamePieceRequest () {
        return new Request() {
            @Override
            public void act () {

            }
            @Override
            public boolean isFinished() {
                return hasGamePiece;
            }
        };
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
    public double getEndEffectorDemand(){
        return mPeriodicIO.demand;
    }
    
    @Log
    public double getEndEffectorVoltage(){
        return mPeriodicIO.voltage;
    }
    
    @Log
    public double getEndEffectorCurrent(){
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
        SmartDashboard.putBoolean("Beam Break", mPeriodicIO.beamBreak); 
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