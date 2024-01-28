// package com.team8013.lib.drivers;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.DemandType;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.sensors.CANCoder;
// import com.team254.lib.drivers.TalonFXFactory;
// import com.team8013.frc2023.Constants;
// import com.team8013.lib.math.Conversions;
// import com.team8013.lib.util.CTREConfigs;
// import com.team8013.lib.util.CTREModuleState;
// import com.team8013.lib.util.SwerveModuleConstants;

// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;

// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
// import com.ctre.phoenix.sensors.CANCoderStatusFrame;

// public class SwerveModuleNonPro {
//     public int moduleNumber;
//     public double angleOffset;
//     private TalonFX mAngleMotor;
//     private TalonFX mDriveMotor;
//     private CANCoder angleEncoder;
//     private double lastAngle;

//     private double anglekP;
//     private double anglekI;
//     private double anglekD;

//     SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

//     public SwerveModuleNonPro(int moduleNumber, SwerveModuleConstants moduleConstants){
//         this.moduleNumber = moduleNumber;
//         angleOffset = moduleConstants.angleOffset;
        
//         /* Angle Encoder Config */
//         //TODO: 
//         angleEncoder = new CANCoder(moduleConstants.cancoderID, "canivore1");
//         configAngleEncoder();
//         angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 300); //originally 255
//         angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 300); //originally 255

//         /* Angle Motor Config */
//         mAngleMotor = TalonFXFactory.createDefaultTalon(moduleConstants.angleMotorID);
//         configAngleMotor();
//         TalonFXConfiguration angleConfiguration = CTREConfigs.swerveAngleFXConfig();
//         anglekP = angleConfiguration.slot0.kP;
//         anglekI = angleConfiguration.slot0.kI;
//         anglekD = angleConfiguration.slot0.kD;

//         /* Drive Motor Config */
//         mDriveMotor = TalonFXFactory.createDefaultTalon(moduleConstants.driveMotorID);
//         configDriveMotor();

//         lastAngle = getState().angle.getDegrees();
//     }

//     public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
//         desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

//         if(isOpenLoop){
//             double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
//             mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
//         }
//         else {
//             double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
//             mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
//         }

//         double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
//         mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.SwerveConstants.angleGearRatio)); 
//         lastAngle = angle;
//     }

//     public void resetToAbsolute(){
//         double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, Constants.SwerveConstants.angleGearRatio);
//         mAngleMotor.setSelectedSensorPosition(absolutePosition);
//     }

//     private void configAngleEncoder(){        
//         angleEncoder.configFactoryDefault();
//         angleEncoder.configAllSettings(CTREConfigs.swerveCancoderConfig());
//     }

//     private void configAngleMotor(){
//         mAngleMotor.configFactoryDefault();
//         mAngleMotor.configAllSettings(CTREConfigs.swerveAngleFXConfig());
//         mAngleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
//         mAngleMotor.setNeutralMode(Constants.SwerveConstants.angleNeutralMode);
//         resetToAbsolute();
//     }

//     private void configDriveMotor(){        
//         mDriveMotor.configFactoryDefault();
//         mDriveMotor.configAllSettings(CTREConfigs.swerveDriveFXConfig());
//         mDriveMotor.setInverted(Constants.SwerveConstants.driveMotorInvert);
//         mDriveMotor.setNeutralMode(Constants.SwerveConstants.driveNeutralMode);
//         mDriveMotor.setSelectedSensorPosition(0);
//     }

//     public void updateAnglePID(double kP, double kI, double kD) {
//         if (anglekP != kP) {
//             anglekP = kP;
//             mAngleMotor.config_kP(0, anglekP, Constants.kLongCANTimeoutMs);
//         }
//         if (anglekI != kI) {
//             anglekI = kI;
//             mAngleMotor.config_kI(0, anglekI, Constants.kLongCANTimeoutMs);
//         }
//         if (anglekD != kP) {
//             anglekD = kD;
//             mAngleMotor.config_kD(0, anglekD, Constants.kLongCANTimeoutMs);        
//         }
//     }

//     public double[] getAnglePIDValues() {
//         double[] values = {anglekP, anglekI, anglekD};
//         return values;
//     }

//     public Rotation2d getCanCoder(){
//         return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
//     }

//     public double getTargetAngle() {
//         return lastAngle;
//     }

//     public SwerveModuleState getState(){
//         double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
//         Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.SwerveConstants.angleGearRatio));
//         return new SwerveModuleState(velocity, angle);
//     }

//         /**
//      * Returns the current position of the module.
//      *
//      * @return The current position of the module.
//      */
//     public SwerveModulePosition getPosition() {
//         return new SwerveModulePosition(getDrivePosition()
//             , Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.SwerveConstants.angleGearRatio)));
//     //Potential Breaking Point^
//     }

//     public double getDrivePosition(){
//         return Conversions.falconToMeters(mDriveMotor.getSensorCollection().getIntegratedSensorPosition(), 
//         1/Constants.SwerveConstants.driveGearRatio, Constants.SwerveConstants.wheelCircumference)*1.02166;  // old: divide by 44.5;
//     }


//     // public double getDriveDistance() {
//     //     return getStateDistance();
//     // }

//     // default SwerveModulePosition getPosition() {
//     //     return new SwerveModulePosition(getDriveDistance(), Rotation2d.fromRadians(angleEncoder.getPosition()));
//     // }

    
// }
