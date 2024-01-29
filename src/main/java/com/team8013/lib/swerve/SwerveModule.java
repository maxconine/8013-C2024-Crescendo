package com.team8013.lib.swerve;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team8013.frc2024.Constants;
import com.team8013.frc2024.Ports;
import com.team8013.frc2024.subsystems.Subsystem;
import com.team8013.lib.Conversions;
import com.team8013.lib.Util;
import com.team8013.lib.logger.Log;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends Subsystem {

    private mPeriodicIO mPeriodicIO = new mPeriodicIO();

    private final int kModuleNumber;
    private final double kAngleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private ModuleState targetModuleState;


    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.kModuleNumber = moduleNumber;
        kAngleOffset = moduleConstants.angleOffset;

        // Absolute encoder config
        angleEncoder = new CANcoder(moduleConstants.cancoderID, Ports.CANBUS);
        angleEncoder.getConfigurator().apply(Constants.SwerveConstants.swerveCancoderConfig());

        // Angle motor config
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, Ports.CANBUS);
        mAngleMotor.getConfigurator().apply(Constants.SwerveConstants.swerveAngleFXConfig());
        mAngleMotor.setPosition(0);

        // Drive motor config
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, Ports.CANBUS);
        mDriveMotor.getConfigurator().apply(Constants.SwerveConstants.swerveDriveFXConfig());
        mDriveMotor.setPosition(0.0);

        resetToAbsolute();
    }

    public void setDesiredState(ModuleState desiredState, boolean isOpenLoop) {
        targetModuleState = desiredState;

        double targetAngle = targetModuleState.angle.getDegrees();
        Rotation2d currentAngle = Rotation2d.fromDegrees(mPeriodicIO.rotationPosition);


        if(isOpenLoop) {
            mPeriodicIO.targetVelocity = targetModuleState.speedMetersPerSecond;
        } else {
            mPeriodicIO.targetVelocity = Util.limit(targetModuleState.speedMetersPerSecond, Constants.SwerveConstants.maxAttainableSpeed);
        }

        if (Util.shouldReverse(Rotation2d.fromDegrees(targetAngle), currentAngle)) {
            mPeriodicIO.targetVelocity = -mPeriodicIO.targetVelocity;
            targetAngle += 180.0;
        }

        targetAngle = Util.placeInAppropriate0To360Scope(getCurrentUnboundedDegrees(), targetAngle);

        mPeriodicIO.rotationDemand = Conversions.degreesToRotation(targetAngle,
                Constants.SwerveConstants.angleGearRatio); //this is a duplicate

        if (isOpenLoop) {
            mPeriodicIO.driveControlMode = ControlModeState.PercentOutput;
            mPeriodicIO.driveDemand = mPeriodicIO.targetVelocity / Constants.SwerveConstants.maxSpeed;
        } else {
            mPeriodicIO.driveControlMode = ControlModeState.Velocity;
            mPeriodicIO.driveDemand = Conversions.MPSToRPS(mPeriodicIO.targetVelocity,
                    Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
        }
    }

    public void resetToAbsolute() {
        double angle = Util.placeInAppropriate0To360Scope(mPeriodicIO.rotationPosition, getCanCoder().getDegrees() - kAngleOffset);
        double absolutePosition = Conversions.degreesToRotation(angle, Constants.SwerveConstants.angleGearRatio);
        mAngleMotor.setPosition(absolutePosition);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(Util.placeInAppropriate0To360Scope(angleEncoder.getAbsolutePosition().getValueAsDouble()*360, angleEncoder.getAbsolutePosition().getValueAsDouble()*360));
    }

    public ModuleState getState() {
        return new ModuleState(mPeriodicIO.drivePosition, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(mPeriodicIO.rotationPosition), mPeriodicIO.velocity);
    }

    @Override
    public synchronized void readPeriodicInputs() {

        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.velocity = Conversions.RPSToMPS(mDriveMotor.getRotorVelocity().getValue(),
                Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);

        mPeriodicIO.rotationPosition = Conversions.rotationsToDegrees(mAngleMotor.getRotorPosition().getValue(),
                Constants.SwerveConstants.angleGearRatio);

        mPeriodicIO.drivePosition = Conversions.rotationsToMeters(mDriveMotor.getRotorPosition().getValue(),
                Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);;
    }

    @Override
    public synchronized void writePeriodicOutputs() {

        double targetAngle = targetModuleState.angle.getDegrees();
        Rotation2d currentAngle = Rotation2d.fromDegrees(mPeriodicIO.rotationPosition);
        if (Util.shouldReverse(Rotation2d.fromDegrees(targetAngle), currentAngle)) {
            mPeriodicIO.targetVelocity = -mPeriodicIO.targetVelocity;
            targetAngle += 180.0;
        }
        targetAngle = Util.placeInAppropriate0To360Scope(getCurrentUnboundedDegrees(), targetAngle);

        mPeriodicIO.rotationDemand = Conversions.degreesToRotation(targetAngle,
                Constants.SwerveConstants.angleGearRatio);

        mAngleMotor.setControl(new PositionDutyCycle(mPeriodicIO.rotationDemand));

        if (mPeriodicIO.driveControlMode == ControlModeState.Velocity) {
            mDriveMotor.setControl(new VelocityTorqueCurrentFOC(mPeriodicIO.driveDemand));
        } else {
            mDriveMotor.setControl(new DutyCycleOut(mPeriodicIO.driveDemand, true, false, false, false));
        }
    }

    public static class SwerveModuleConstants {
        public final int driveMotorID;
        public final int angleMotorID;
        public final int cancoderID;
        public final double angleOffset;

        public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, double angleOffset) {
            this.driveMotorID = driveMotorID;
            this.angleMotorID = angleMotorID;
            this.cancoderID = canCoderID;
            this.angleOffset = angleOffset;
        }
    }

    public static class mPeriodicIO {
        // Inputs
        public double timestamp = 0.0;
        public double targetVelocity = 0.0;
        public double rotationPosition = 0.0;
        public double drivePosition = 0.0;
        public double velocity = 0.0;

        // Outputs
        public ControlModeState driveControlMode = ControlModeState.PercentOutput;
        public double rotationDemand;
        public double driveDemand;
    }

    private enum ControlModeState{
        PercentOutput,
        Velocity
    }

    public int moduleNumber() {
        return kModuleNumber;
    }

    public double angleOffset() {
        return kAngleOffset;
    }

    public double getDriveMotorCurrent() {
        return mDriveMotor.getStatorCurrent().getValue();
    }

    public void setDriveNeutralBrake(boolean wantBrake) {
        if (wantBrake) {
            mDriveMotor.setNeutralMode(NeutralModeValue.Brake);
            mAngleMotor.setNeutralMode(NeutralModeValue.Coast);
        } else {
            
            mDriveMotor.setNeutralMode(NeutralModeValue.Coast);
            // Brake angle motors when coasting drive
            mAngleMotor.setNeutralMode(NeutralModeValue.Brake);
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Module" + kModuleNumber + " Angle Position", mPeriodicIO.rotationPosition);
        SmartDashboard.putNumber("Module" + kModuleNumber + " Drive Position", mPeriodicIO.drivePosition);
        SmartDashboard.putNumber("Module" + kModuleNumber + " Velocity", mPeriodicIO.velocity);
    }

    @Log
    public double getTargetAngle() {
        return Conversions.rotationsToDegrees(mPeriodicIO.rotationDemand,
                Constants.SwerveConstants.angleGearRatio);
    }

    @Log
    public double getTargetVelocity() {
        return mPeriodicIO.targetVelocity;
    }

    @Log
    public double getCurrentSpeed() {
        return getState().speedMetersPerSecond;
    }

    @Log
    public double getCurrentUnboundedDegrees() {
        return mPeriodicIO.rotationPosition;
    }

    @Log
    public double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}