package com.team8013.frc2024;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.team8013.frc2024.subsystems.Drive.KinematicLimits;
import com.team8013.lib.Conversions;
import com.team8013.lib.swerve.SwerveModule.SwerveModuleConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

import com.team8013.lib.swerve.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.util.Units;

public class Constants {

    // toggle constants between comp bot and practice bot ("epsilon")
    public static boolean isBeta = false;
    public static boolean isComp = true;

    public static boolean isCompBot() {
        return isComp;
    }

    public static boolean isBetaBot() {
        return isBeta;
    }

    // Disables extra smart dashboard outputs that slow down the robot
    public static final boolean disableExtraTelemetry = true;

    public static final boolean isManualControlMode = false;

    // robot loop time
    public static final double kLooperDt = 0.02;

    /* Control Board */
    public static final double kTriggerThreshold = 0.2;

    public static final double stickDeadband = 0.05;
    public static final int leftXAxis = 0;
    public static final int leftYAxis = 1;
    public static final int rightXAxis = 3;
    public static final int rightYAxis = 4;

    public static final class SwerveConstants {
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.25);
        public static final double wheelBase = Units.inchesToMeters(20.25);

        public static final double wheelDiameter = Units.inchesToMeters(3.85);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        // this is for the comp bot
        // public static final double driveGearRatio = 6.75; //flipped gear ratio
        // https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options
        // public static final double angleGearRatio = 15.43; //8:32:24--14:72 = 15.43
        // ratio

        public static final double driveGearRatio = ((5.3 / 1.07) / 1.04); // (((5.3 / 1.07)/2)*0.952); //((5.3 / 1.07)
                                                                           // / 1.04);// // TODO: This needs to be done
                                                                           // // 6.525 * //might just be 2
                                                                           // 8.215 / 8; // 6.55
        public static final double angleGearRatio = 21.4285714;// (150/7);// 10.29; // 72:14:24:12

        // 43.75 - number to divide driven distance by

        // public static final edu.wpi.first.math.geometry.Translation2d[]
        // swerveModuleLocations = {
        // new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, trackWidth /
        // 2.0),
        // new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, -trackWidth /
        // 2.0),
        // new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, trackWidth /
        // 2.0),
        // new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, -trackWidth /
        // 2.0)
        // };

        public static final edu.wpi.first.math.geometry.Translation2d[] swerveModuleLocations = {
                new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
                new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, trackWidth / 2.0)
        };

        public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(swerveModuleLocations);

        // public static final SwerveDriveKinematics swerveKinematics = new
        // SwerveDriveKinematics(swerveModuleLocations);

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 62;
        public static final int drivePeakCurrentLimit = 65;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.32 / 12);
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.8; // meters per second MAX : 5.02 m/s
        public static final double maxAngularVelocity = 8.0;

        public static final double maxAttainableSpeed = maxSpeed * 0.85; // Max out at 85% to make sure speeds are
                                                                         // attainable (4.6 mps)

        /* Angle Motor PID Values */
        public static final double angleKP = 0.6; // 0.3 originally
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 4.0;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 12.0 / Conversions.MPSToRPS(maxSpeed, wheelCircumference, driveGearRatio);

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Motor Inverts */
        public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive; // false TODO:
                                                                                                      // check if
        // this is right
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive; // true;

        // THIS MIGHT HAVE FIXED IT!

        /* Angle Encoder Invert */
        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive; // false
        // TODO:is
        // this
        // right?
        // Changed above inverts but not this yet..

        /* Controller Invert */
        public static final boolean invertYAxis = false;
        public static final boolean invertRAxis = false;
        public static final boolean invertXAxis = true;

        public static final KinematicLimits kUncappedLimits = new KinematicLimits();
        static {
            kUncappedLimits.kMaxDriveVelocity = maxSpeed;
            kUncappedLimits.kMaxAccel = Double.MAX_VALUE;
            kUncappedLimits.kMaxAngularVelocity = maxAngularVelocity;
            kUncappedLimits.kMaxAngularAccel = Double.MAX_VALUE;
        }

        public static final KinematicLimits kScoringLimits = new KinematicLimits();
        static {
            kScoringLimits.kMaxDriveVelocity = 2.0;
            kScoringLimits.kMaxAccel = Double.MAX_VALUE;
            kScoringLimits.kMaxAngularVelocity = Math.PI; // Rad/Sec
            kScoringLimits.kMaxAngularAccel = 10 * Math.PI; // 2 * Math.PI;
        }

        public static final KinematicLimits kLoadingStationLimits = new KinematicLimits();
        static {
            kLoadingStationLimits.kMaxDriveVelocity = 1.5;
            kLoadingStationLimits.kMaxAccel = Double.MAX_VALUE;
            kLoadingStationLimits.kMaxAngularVelocity = maxAngularVelocity;
            kLoadingStationLimits.kMaxAngularAccel = Double.MAX_VALUE;
        }

        public static final KinematicLimits kAutoLimits = new KinematicLimits();
        static {
            kAutoLimits.kMaxDriveVelocity = maxAttainableSpeed;
            kAutoLimits.kMaxAccel = Double.MAX_VALUE;
            kAutoLimits.kMaxAngularVelocity = Double.MAX_VALUE; // Rad/Sec
            kAutoLimits.kMaxAngularAccel = Double.MAX_VALUE; // 2 * Math.PI;
        }

        /***
         * MODULE SPECIFIC CONSTANTS **
         * 
         * which way to zero modules?
         * foreward, right
         * 90: x+ y- (doesnt turn)
         * 180 x-, y+ (turns opposite)
         * 270 x+, y+ (doesn't turn)
         * 
         * Zero them so that odometry is x positive going forwards and y positive going
         * left
         * 
         * 
         */

        /*** MODULE SPECIFIC CONSTANTS ***/
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final double betaAngleOffset = 40.07;
            public static final double compAngleOffset = 40.07;

            public static SwerveModuleConstants SwerveModuleConstants() {
                return new SwerveModuleConstants(Ports.FL_DRIVE, Ports.FL_ROTATION, Ports.FL_CANCODER,
                        isComp ? compAngleOffset : betaAngleOffset);
            }
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final double betaAngleOffset = 171.21;
            public static final double compAngleOffset = 171.21;

            public static SwerveModuleConstants SwerveModuleConstants() {
                return new SwerveModuleConstants(Ports.FR_DRIVE, Ports.FR_ROTATION, Ports.FR_CANCODER,
                        isComp ? compAngleOffset : betaAngleOffset);
            }
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final double betaAngleOffset = 182.46;
            public static final double compAngleOffset = 182.46;

            public static SwerveModuleConstants SwerveModuleConstants() {
                return new SwerveModuleConstants(Ports.BL_DRIVE, Ports.BL_ROTATION, Ports.BL_CANCODER,
                        isComp ? compAngleOffset : betaAngleOffset);
            }
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final double betaAngleOffset = 277.82;
            public static final double compAngleOffset = 277.82;

            public static SwerveModuleConstants SwerveModuleConstants() {
                return new SwerveModuleConstants(Ports.BR_DRIVE, Ports.BR_ROTATION, Ports.BR_CANCODER,
                        isComp ? compAngleOffset : betaAngleOffset);
            }
        }

        // public static Pigeon2Configuration pigeonConfig(){
        // Pigeon2Configuration config = new Pigeon2Configuration();

        // }

        public static TalonFXConfiguration swerveDriveFXConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits.SupplyCurrentLimitEnable = Constants.SwerveConstants.driveEnableCurrentLimit;
            config.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.driveContinuousCurrentLimit;
            config.CurrentLimits.SupplyCurrentThreshold = Constants.SwerveConstants.drivePeakCurrentLimit;
            config.CurrentLimits.SupplyTimeThreshold = Constants.SwerveConstants.drivePeakCurrentDuration;

            config.Voltage.PeakForwardVoltage = 12.0;
            config.Voltage.PeakReverseVoltage = -12.0;

            config.Slot0.kP = Constants.SwerveConstants.driveKP;
            config.Slot0.kI = Constants.SwerveConstants.driveKI;
            config.Slot0.kD = Constants.SwerveConstants.driveKD;
            // config.Slot0.kV = Constants.SwerveConstants.driveKF;

            config.MotorOutput.NeutralMode = Constants.SwerveConstants.driveNeutralMode;
            config.MotorOutput.Inverted = Constants.SwerveConstants.driveMotorInvert;

            config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;
            config.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;
            return config;
        }

        public static TalonFXConfiguration swerveAngleFXConfig() {
            TalonFXConfiguration angleConfig = new TalonFXConfiguration();
            // TODO: do any of these configs even matter?
            angleConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.SwerveConstants.angleEnableCurrentLimit;
            angleConfig.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.angleContinuousCurrentLimit;
            angleConfig.CurrentLimits.SupplyCurrentThreshold = Constants.SwerveConstants.anglePeakCurrentLimit;
            angleConfig.CurrentLimits.SupplyTimeThreshold = Constants.SwerveConstants.anglePeakCurrentDuration;

            angleConfig.Slot0.kP = Constants.SwerveConstants.angleKP;
            angleConfig.Slot0.kI = Constants.SwerveConstants.angleKI;
            angleConfig.Slot0.kD = Constants.SwerveConstants.angleKD;
            angleConfig.Slot0.kV = Constants.SwerveConstants.angleKF;

            angleConfig.MotorOutput.NeutralMode = Constants.SwerveConstants.angleNeutralMode;
            angleConfig.MotorOutput.Inverted = Constants.SwerveConstants.angleMotorInvert;

            // angleConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
            // Constants.SwerveConstants.openLoopRamp;
            // angleConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod =
            // Constants.SwerveConstants.openLoopRamp;
            return angleConfig;
        }

        public static CANcoderConfiguration swerveCancoderConfig() {
            CANcoderConfiguration config = new CANcoderConfiguration();
            config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1; // TODO: PROBLEM
            config.MagnetSensor.SensorDirection = Constants.SwerveConstants.canCoderInvert;
            // config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
            // config.sensorDirection = Constants.SwerveConstants.canCoderInvert;
            // config.initializationStrategy =
            // SensorInitializationStrategy.BootToAbsolutePosition;
            // config.sensorTimeBase = SensorTimeBase.PerSecond;
            return config;
        }
    }

    public static final class SnapConstants {
        public static final double kP = 6.0; // was 6
        public static final double kI = 0.5; // was .5
        public static final double kD = 0.2; // was .2
        public static final double snapTimeout = 0.25;
        public static final double snapEpsilon = 1.0;

    }

    public static final class AutoConstants {

        public static final double kPXController = 6.7;
        public static final double kPYController = 6.7;

        public static final double kDXController = 0.0;
        public static final double kDYController = 0.0;

        public static final double kPThetaController = 2.75; // was 2, changed to 4

        // Constraint for the motion profilied robot angle controller (Radians)
        public static final double kMaxAngularSpeed = 2.0 * Math.PI;
        public static final double kMaxAngularAccel = 2.0 * Math.PI * kMaxAngularSpeed;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeed, kMaxAngularAccel);

        // Static factory for creating trajectory configs
        public static TrajectoryConfig createConfig(double maxSpeed, double maxAccel, double startSpeed,
                double endSpeed) {
            TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAccel);
            config.setStartVelocity(startSpeed);
            config.setEndVelocity(endSpeed);
            config.addConstraint(new CentripetalAccelerationConstraint(10.0));
            return config;
        }
    }

    public static final class VisionAlignConstants {
        public static final double kP = 6.37;
        public static final double kI = 0.0;
        public static final double kD = 0.10;
        public static final double kTimeout = 0.25;
        public static final double kEpsilon = 5.0;

        // Constraints for the profiled angle controller
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.0 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 10.0 * Math.PI;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        /* April Tag Chase */

        public static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 1);
        public static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 1);
        public static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

        public static final int TAG_TO_CHASE = 1;
        public static final Transform3d TAG_TO_GOAL = new Transform3d(
                new Translation3d(0.0, 0.0, 0.0),
                new Rotation3d(0.0, 0.0, Math.PI));

        public static final TrajectoryConfig TAG_TRAJECTORY_CONFIG = Constants.AutoConstants.createConfig(2.5, 2.0, 0.0,
                0.0);

        public static final double POSITION_OFF = 0.1;

        /**
         * Standard deviations of model states. Increase these numbers to trust your
         * model's state estimates less. This matrix is in the form [x, y, theta]ᵀ,
         * with units in meters and radians, then meters.
         */
        public static final Vector<N3> STATE_STDS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

        /**
         * Standard deviations of the vision measurements. Increase these numbers to
         * trust global measurements from vision less. This matrix is in the form
         * [x, y, theta]ᵀ, with units in meters and radians.
         */
        public static final Vector<N3> VISION_STDS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));

    }

    public static final class MacAddressConstants {
        public static final byte[] COMP_ADDRESS = new byte[] {
                // values are for comp -> 00:80:2f:35:b8:ca
                (byte) 0x00, (byte) 0x80, (byte) 0x2f, (byte) 0x35, (byte) 0xb8, (byte) 0xca
        };

        public static final byte[] BETA_ADDRESS = new byte[] {
                // values are for beta -> 00:80:2f:34:0B:9B
                (byte) 0x00, (byte) 0x80, (byte) 0x2f, (byte) 0x34, (byte) 0x0b, (byte) 0x9b
        };
    }

    // public static final class VisionConstants {
    // public static final LimelightConstants kLimelightConstants = new
    // LimelightConstants();
    // static {
    // kLimelightConstants.kName = "Limelight";
    // kLimelightConstants.kTableName = "limelight";
    // kLimelightConstants.kHeight = 0.79; // meters
    // kLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(0.0);
    // }

    // public static final double kHorizontalFOV = 59.6; // degrees
    // public static final double kVerticalFOV = 49.7; // degrees
    // public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

    // // lookahead time
    // public static final double kLookaheadTime = 0.0; // 1.10 as latest

    // /* Goal Tracker Constants */
    // public static final double kMaxTrackerDistance = 8.0;
    // public static final double kMaxGoalTrackAge = 10.0;
    // public static final double kMaxGoalTrackSmoothingTime = 1.5;
    // public static final double kCameraFrameRate = 90.0;

    // public static final double kTrackStabilityWeight = 0.0;
    // public static final double kTrackAgeWeight = 10.0;
    // public static final double kTrackSwitchingWeight = 100.0;

    // public static final int kDefaultPipeline = 0;
    // public static final double kGoalHeight = 2.63; // meters
    // public static final double kGoalRadius = Units.inchesToMeters(.5); // meters
    // }

    /*** SUBSYSTEM CONSTANTS ***/

    public static final class PivotConstants {
        public static final double kStatorCurrentLimit = 80.0;
        public static final double CANCODER_OFFSET = 106.1; // -4.8 so it never gets to -360 and breaks now it's 4.8 on
                                                            // 3/27
        public static final double kPositionError = 2; // 2 degrees of error

        public static final double gravityFeedforward = 0.0; // idk how this works

        public static final double PivotGearRatio = (25) * (74 / 18);// 25:1 74:18 revolutions of the pivot per 1
                                                                     // rotation of the motor

        public static final int kMinAngle = 5; // deg
        public static final int kMaxAngle = 95; // deg TODO: SET THESE WHEN CONFIGING MOTOR (convert to rotations)

        /* State Positions */
        // public static final double kFloorIntakeAngle = 0;
        public static final double kSourceIntakeAngle = 68;
        public static final double kSourceLoadShooterAngle = 41; // if anything, lower
        public static final double kStowAngle = 4.8;
        public static final double kAmpScoreAngle = 88; // was 88

        // SHOOTING ANGLES
        public static final double kShootAgainstSubwooferAngle = 56 + 2.5 + 1.25 + .15 + 0.25 + 0.3 + 0.75 - 1; // changed
                                                                                                            // from 55 //TODO: UNDO THE -1 BEFORE HOUSTON!
        public static final double kShootAgainstPodiumAngle = 36.5;
        public static final double kPassNoteFromMidAngle = 56;

        // Autos
        public static final double kStage2PieceAngle = 44.5; //46 at dcmp?
        public static final double kMid2PieceAngle = 55; // 53 - 1;
        public static final double kAmp2PieceAngle = 39;

        public static final double kShootLoadAngle = 56; // changed from 54

        /* CLIMB CONSTANTS */
        public static final double kClimbInitAngle1 = 63; // deg
        public static final double kClimbInitAngle2 = 68; // deg
        public static final double kPullOntoChainAngle1 = 20;
        public static final double kPullOntoChainAngle2 = 6.75; // once elevator is down, goto this angle
        public static final double kExtendOffChainAngle1 = 16.8; // Once chain hooked go up to this angle and wait for
                                                                 // release
        public static final double kExtendOffChainAngle2 = 32; // Angle before abrupt flip over to trap
        public static final double kExtendToScoreTrapAngle1 = 67; // angle when pressed up against trap wall
        public static final double kExtendToScoreTrapAngle2 = 97; // angle when pressed up against trap wall

        /* CLIMB DOWN CONSTANTS */
        public static final double kDeclimbAngle1 = 50.7;
        public static final double kDeclimbAngle2 = 31.7;
        public static final double kDeclimbAngle3 = 7.5; // was 8.2
        public static final double kDeclimbAngle4 = 78;

        public static final double kIntakeCruiseVelocity = 40;
        public static final double kIntakeAcceleration = 80;

        public static CANcoderConfiguration pivotCancoderConfig() {
            CANcoderConfiguration config = new CANcoderConfiguration();
            config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
            config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
            return config;
        }

        public static TalonFXConfiguration pivotFastMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            // TODO: do any of these configs even matter?
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 25; // start off pretty low
            config.CurrentLimits.SupplyCurrentThreshold = 40;
            config.CurrentLimits.SupplyTimeThreshold = 0.1;

            config.Slot0.kP = 0.7;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kV = 0.0;

            config.MotionMagic.MotionMagicCruiseVelocity = 150;
            config.MotionMagic.MotionMagicExpo_kA = 0.7;
            config.MotionMagic.MotionMagicAcceleration = 120;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            return config;
        }

        public static TalonFXConfiguration pivotSlowMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            // TODO: do any of these configs even matter?
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 30; // start off pretty low
            config.CurrentLimits.SupplyCurrentThreshold = 40;
            config.CurrentLimits.SupplyTimeThreshold = 0.1;

            config.Slot0.kP = 0.7;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kV = 0.0;

            config.MotionMagic.MotionMagicCruiseVelocity = 35;
            config.MotionMagic.MotionMagicExpo_kA = 0.7;
            config.MotionMagic.MotionMagicAcceleration = 80;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            return config;
        }

        public static TalonFXConfiguration pivotCurlMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            // TODO: do any of these configs even matter?
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 30; // start off pretty low
            config.CurrentLimits.SupplyCurrentThreshold = 40;
            config.CurrentLimits.SupplyTimeThreshold = 0.1;

            config.Slot0.kP = 0.7;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kV = 0.0;

            config.MotionMagic.MotionMagicCruiseVelocity = 15; // TODO: change this
            config.MotionMagic.MotionMagicExpo_kA = 0.7;
            config.MotionMagic.MotionMagicAcceleration = 30;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            return config;
        }

    }

    public static final class WristConstants {
        public static final double CANCODER_OFFSET = -91.56+60.56;// +4;//86.3 + 58 + 4; // +3.3 so it never gets
                                                            // there

        public static final double kGearRatio = 25; // radians per rotation

        public static final double kMinPosition = 0; // degrees
        public static final double kMaxPosition = 200; // degrees

        public static final double kSourceIntakeAngle = 294;
        public static final double kStowAngle = 155;
        public static final double kAmpScoreAngle = 164; // was 169
        public static final double kloadShooterAngle = 118;// 118.8;

        public static final double kShootAngle = 118; //cancoder should rest at 118.8, so that when shooting the wrist is pulled down against elevator

        public static final double kClimbAngle1 = 140;
        public static final double kClimbFirstPressAngle = 192; //lowered from 200
        public static final double kClimbSecondPressAngle = 218; //lowered from 225
        // public static final double kClimbAngle3 = 165;
        public static final double kClimbScoreInTrapAngle = 160; // ~200?

        public static final double kIntakeCruiseVelocity = 50;
        public static final double kIntakeAcceleration = 120;

        public static TalonFXConfiguration wristMotorClimbConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 15; // start off pretty low
            config.CurrentLimits.SupplyCurrentThreshold = 20;
            config.CurrentLimits.SupplyTimeThreshold = 0.1;

            config.Slot0.kP = 0.6;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kV = 0.0;

            config.MotionMagic.MotionMagicCruiseVelocity = 80;
            config.MotionMagic.MotionMagicExpo_kA = 0.2;
            config.MotionMagic.MotionMagicAcceleration = 120;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // down to intake is increasing, up
                                                                            // to load
            // is decreasing

            return config;
        }

        public static TalonFXConfiguration wristMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 15; // start off pretty low
            config.CurrentLimits.SupplyCurrentThreshold = 20;
            config.CurrentLimits.SupplyTimeThreshold = 0.1;

            config.Slot0.kP = 0.6;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kV = 0.0;

            config.MotionMagic.MotionMagicCruiseVelocity = 100;
            config.MotionMagic.MotionMagicExpo_kA = 0.2;
            config.MotionMagic.MotionMagicAcceleration = 170;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // down to intake is increasing, up
                                                                            // to load
            // is decreasing

            return config;
        }

        public static CANcoderConfiguration wristCancoderConfig() {
            CANcoderConfiguration config = new CANcoderConfiguration();
            config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
            config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
            return config;
        }
    }

    public static final class ElevatorConstants {
        public static final int kMaxVoltage = 12;
        public static final double kGearRatio = 12;
        public static final double kWheelCircumference = Conversions.inchesToMeters(1.625) * Math.PI;
        public static final double kPositionError = Conversions.inchesToMeters(0.5);

        public static final double kMinHeight = 0; // meters
        public static final double kMaxHeight = 0.7;

        public static final double kStowHeight = 0.018;
        public static final double kAmpScoreHeight = 0.22 + Conversions.inchesToMeters(1);

        /* SHOOTING */
        public static final double kloadShooterInitialHeight = 0.32 + Conversions.inchesToMeters(1.7);
        public static final double kloadShooterFinalHeight = 0.034 + Conversions.inchesToMeters(5.5);
        public static final double kShootHeight = 0.26;

        /* INTAKING */
        public static final double kIntakeCruiseVelocity = 90;
        public static final double kIntakeAcceleration = 120;

        public static final double kFloorIntakeHeight = 0.29;
        public static final double kSourceIntakeHeight = 0.064;
        public static final double kSourceLoadShooterHeight = 0.22;

        /* CLIMB */
        public static final double kClimbInitHeight = 0.32 + Conversions.inchesToMeters(4.75); // initial height going
                                                                                               // up
        public static final double kMaxClimbInitHeight = 0.32 + Conversions.inchesToMeters(8); // TODO: set this
        // to chain
        public static final double kPullOntoChainHeight = Conversions.inchesToMeters(0.25); // height of the elevator when transfering chain

        public static final double kExtendOffChain1 = 0.054;
        public static final double kExtendOffChain2 = 0.126;
        public static final double kExtendOffChain3 = 0.26 - Conversions.inchesToMeters(1); // to go within height
                                                                                            // limits
        public static final double kExtendToScoreTrapHeight = 0.447 - Conversions.inchesToMeters(2.5); // height of the
                                                                                                       // elvator when
                                                                                                       // scoring in the
                                                                                                       // trap

        /* De Climb */
        public static final double kDeclimbHeight1 = 0.267;
        public static final double kDeclimbHeight2 = 0.06;
        public static final double kDeclimbHeight3 = 0.02;
        public static final double kDeclimbHeight4 = 0.32 + Conversions.inchesToMeters(2);

        public static TalonFXConfiguration elevatorFastMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 30; // start off pretty low
            config.CurrentLimits.SupplyCurrentThreshold = 20;
            config.CurrentLimits.SupplyTimeThreshold = 0.1;

            config.Slot0.kP = 0.6;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kV = 0.0;

            config.MotionMagic.MotionMagicCruiseVelocity = 140;
            config.MotionMagic.MotionMagicExpo_kA = 0.3;
            config.MotionMagic.MotionMagicAcceleration = 300;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            return config;
        }

        public static TalonFXConfiguration elevatorSlowMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            // TODO: do any of these configs even matter?
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 30; // start off pretty low
            config.CurrentLimits.SupplyCurrentThreshold = 20;
            config.CurrentLimits.SupplyTimeThreshold = 0.1;

            config.Slot0.kP = 0.6;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kV = 0.0;

            config.MotionMagic.MotionMagicCruiseVelocity = 140; // was 50 for 1st comp
            config.MotionMagic.MotionMagicExpo_kA = 0.2;
            config.MotionMagic.MotionMagicAcceleration = 140;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            return config;
        }

        public static TalonFXConfiguration elevatorCurlMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            // TODO: do any of these configs even matter?
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 35; // start off pretty low
            config.CurrentLimits.SupplyCurrentThreshold = 30;
            config.CurrentLimits.SupplyTimeThreshold = 0.1;

            config.Slot0.kP = 0.6;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kV = 0.0;

            config.MotionMagic.MotionMagicCruiseVelocity = 140; // TODO: change this
            config.MotionMagic.MotionMagicExpo_kA = 0.2;
            config.MotionMagic.MotionMagicAcceleration = 200;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            return config;
        }

        public static final double[][] groundIntakeWristPositionsOut = {
                // @0 --> position of elevator (in meters)
                // @1 --> position of wrist (in degrees)
                // @2 --> position of the pivot(in degrees)
                { 0.004, 335, 5 },
                { 0.03, 337, 6 },
                { 0.052, 340, 8 },
                { 0.075, 341, 8 },
                { 0.1, 343, 8 },
                { 0.125, 345, 8 },
                { 0.15, 348, 7 },
                { 0.175, 354, 7 },
                { 0.2, 356, 7 },
                { 0.215, 358, 7 },
                { 0.23, 359.7 + 3, 7 },
                { 0.235, 359.7 + 4, 7 },
                { 0.237, 362 + 5, 6 } // really 0.275, but less so that everything else goes into position

        };

        public static final double[][] groundIntakeWristPositionsIn = {
                // @0 --> position of elevator (in meters)
                // @1 --> position of wrist (in degrees)
                // @2 --> position of the pivot(in degrees)
                { 0.004, 190, 4.8 },
                { 0.03, 200, 4.8 },
                { 0.052, 210, 5 },
                { 0.075, 215, 5 },
                { 0.1, 235, 6 },
                { 0.125, 290, 7 },
                { 0.15, 300, 8 },
                { 0.175, 328, 9 },
                { 0.2, 330, 10 },
                { 0.215, 333, 11 },
                { 0.23, 335, 11 },
                { 0.235, 340, 11 },
                { 0.24, 345, 10 } // really 0.275, but less so that everything else goes into position

        };
    }

    public static final class EndEffectorConstants {
        // SHOOTING RPM's
        public static final double kSubwooferRPM = 5000; // 5000
        public static final double kShootFastRPM = 6500; // rpm for passing and shooting from furthur away
        public static final double kPassRPM = 6300;
        // INTAKE/OUTTAKE DEMANDS
        public static final double kSourceIntakeDemand = 0.35;
        public static final double kGroundIntakeDemand = 0.58;
        public static final double kOuttakingDemandTop = -0.50;
        public static final double kOuttakingDemandBottom = -0.55;

        // PID TUNING
        // SUBWOOFER
        public static final double kFFTopSubwoofer = 0.000157; // this value tunes the subwoofer shot
        public static final double kFFBottomSubwoofer = 0.0001566; // this value tunes the subwoofer shot
        // FAST
        public static final double kFFTopFast = 0.0001564; // this value tunes the note passing
        public static final double kFFBottomFast = 0.000156; // this value tunes the note passing

        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;

        public static final double kPSubWof = 0.00022;
        public static final double kPFast = 0.00022;

    }

    public static final class ShooterConstants {
        public static final double kLoadShooterDemand = -0.65;
        public static final double kSlingshotDemand = 0.95;

        public static TalonFXConfiguration shooterMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 30; // start off pretty low
            config.CurrentLimits.SupplyCurrentThreshold = 20;
            config.CurrentLimits.SupplyTimeThreshold = 0.1;

            config.MotionMagic.MotionMagicExpo_kA = 0.2;
            config.MotionMagic.MotionMagicAcceleration = 300;

            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

            return config;
        }
    }

    public static final class ClimberHookConstants {
        public static final double kHookAngle = 88; // degrees
        public static final double kDeclimb1Angle = 88;
        public static final double kUnhookAngle = 0; // makes it so we don't have to worry about resetting it while
                                                     // practicing
        public static final double kMaxAngle = 131;
        public static final double kMinAngle = 0;
        public static final double kGearRatio = 45;

        public static TalonFXConfiguration climberHookMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits.SupplyCurrentLimitEnable = false;
            config.CurrentLimits.SupplyCurrentLimit = 10; // start off pretty low
            config.CurrentLimits.SupplyCurrentThreshold = 20;
            config.CurrentLimits.SupplyTimeThreshold = 0.1;

            config.Slot0.kP = 0.6;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kV = 0.0;

            config.MotionMagic.MotionMagicCruiseVelocity = 160;
            config.MotionMagic.MotionMagicExpo_kA = 0.3;
            config.MotionMagic.MotionMagicAcceleration = 400;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            return config;
        }
    }

    public static final class ControllerConstants {
        public static final boolean isMamboController = true; // this overrides everything
        public static final boolean isControllerOne = true;

        // Controller 1 left side:
        public static final double ControllerOneLeftThrottleZero = -0.125;
        public static final double ControllerOneLeftYawZero = 0.039370;

        public static final double ControllerOneLeftThrottleHigh = 0.787402;
        public static final double ControllerOneLeftThrottleLow = 0.968750;

        public static final double ControllerOneLeftYawHigh = 0.86612;
        public static final double ControllerOneLeftYawLow = 0.77338;

        // Controller 1 right side:
        public static final double ControllerOneRightThrottleZero = 0.055118;
        public static final double ControllerOneRightYawZero = 0.055118;

        public static final double ControllerOneRightYawHigh = 0.866142;
        public static final double ControllerOneRightYawLow = 0.765625;

        public static final double ControllerOneRightThrottleHigh = 0.732283;
        public static final double ControllerOneRightThrottleLow = 0.601563;

        // Controller 2 left side:
        public static final double ControllerTwoLeftThrottleZero = -0.023438;
        public static final double ControllerTwoLeftYawZero = -0.078125;

        public static final double ControllerTwoLeftThrottleHigh = 0.834646;
        public static final double ControllerTwoLeftThrottleLow = 0.867188;

        public static final double ControllerTwoLeftYawHigh = 0.748031;
        public static final double ControllerTwoLeftYawLow = 0.890625;

        // Controller 2 right side:
        public static final double ControllerTwoRightThrottleZero = -0.054688; // high 0.007874
        public static final double ControllerTwoRightYawZero = 0.062992;

        public static final double ControllerTwoRightYawHigh = 0.866142;
        public static final double ControllerTwoRightYawLow = 0.664063;

        public static final double ControllerTwoRightThrottleHigh = 0.669291;
        public static final double ControllerTwoRightThrottleLow = 0.664063;

        // Controller left side:
        public static final double ControllerLeftThrottleZero = isControllerOne ? ControllerOneLeftThrottleZero
                : ControllerTwoLeftThrottleZero;
        public static final double ControllerLeftYawZero = isControllerOne ? ControllerOneLeftYawZero
                : ControllerTwoLeftYawZero;

        public static final double ControllerLeftThrottleHigh = isControllerOne ? ControllerOneLeftThrottleHigh
                : ControllerTwoLeftThrottleHigh;
        public static final double ControllerLeftThrottleLow = isControllerOne ? ControllerOneLeftThrottleLow
                : ControllerTwoLeftThrottleLow;

        public static final double ControllerLeftYawHigh = isControllerOne ? ControllerOneLeftYawHigh
                : ControllerTwoLeftYawHigh;
        public static final double ControllerLeftYawLow = isControllerOne ? ControllerOneLeftYawLow
                : ControllerTwoLeftYawLow;

        // Controller right side:
        public static final double ControllerRightThrottleZero = isControllerOne ? ControllerOneRightThrottleZero
                : ControllerTwoRightThrottleZero;
        public static final double ControllerRightYawZero = isControllerOne ? ControllerOneRightYawZero
                : ControllerTwoRightYawZero;

        public static final double ControllerRightYawHigh = isControllerOne ? ControllerOneRightYawHigh
                : ControllerTwoRightYawHigh;
        public static final double ControllerRightYawLow = isControllerOne ? ControllerOneRightYawLow
                : ControllerTwoRightYawLow;

        public static final double ControllerRightThrottleHigh = isControllerOne ? ControllerOneRightThrottleHigh
                : ControllerTwoRightThrottleHigh;
        public static final double ControllerRightThrottleLow = isControllerOne ? ControllerOneRightThrottleLow
                : ControllerTwoRightThrottleLow;

    }

    // Timeout constants
    public static final int kLongCANTimeoutMs = 100;
    public static final int kCANTimeoutMs = 10;
}
