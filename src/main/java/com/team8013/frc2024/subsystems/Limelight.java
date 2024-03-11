package com.team8013.frc2024.subsystems;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.Util;
import com.team254.lib.vision.TargetInfo;
import com.team8013.frc2024.Constants;
import com.team8013.frc2024.loops.ILooper;
import com.team8013.frc2024.loops.Loop;
import com.team8013.lib.Conversions;
import com.team8013.lib.swerve.ChassisSpeeds;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.team8013.frc2024.regressions.ShooterRegression;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Subsystem for interacting with the Limelight 3
 */
public class Limelight extends Subsystem {

    private static Limelight mInstance;
    private final Drive mSwerve = Drive.getInstance();
    private final ShooterRegression mRegression = new ShooterRegression();

    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController omegaController;
    private Pose3d robotPose;
    boolean isRedAlliance = true;
    private boolean shootAgainstSubwooferSide = false;

    private int mLatencyCounter = 0;

    // distance to target
    
    public Optional<Double> mDistanceToTarget = Optional.empty();

    NetworkTable mNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private boolean mOutputsHaveChanged = true;
    private boolean shootFromPodium = false;

    private final NetworkTableEntry botpose_wpiblue = mNetworkTable.getEntry("botpose_wpiblue");
    private final NetworkTableEntry botpose_wpired = mNetworkTable.getEntry("botpose_wpired");
    private final NetworkTableEntry tTargetID = mNetworkTable.getEntry("tid");

    

    // private final NetworkTableEntry tCameraPose =
    // mNetworkTable.getEntry("camerapose_targetspace"); // makes it work idk

        // private final NetworkTableInstance instance;// =
    // NetworkTableInstance.getDefault(); // changed networktable setup
    // NetworkTable mNetworkTable;// = instance.getTable("Limelight_0"); // changed
    // table name to match limelights name



    private Limelight() {

        // mNetworkTable.getEntry("tv");

        // mNetworkTable.getEntry("ledMode").setNumber(3);

        /* Aril Tag Chase */
        // initializeAprilTagChase();
    }

    public static Limelight getInstance() {
        if (mInstance == null) {
            mInstance = new Limelight();
        }
        return mInstance;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        Loop mLoop = new Loop() {
            @Override
            public void onStart(double timestamp) {
                // RobotState.getInstance().resetVision();
                setLed(LedMode.OFF);
            }

            @Override
            public void onLoop(double timestamp) {
                final double start = Timer.getFPGATimestamp();

                // synchronized (Limelight.this) {
                // List<TargetInfo> targetInfo = getTarget();
                // if (mPeriodicIO.sees_target && targetInfo != null) {
                // // RobotState.getInstance().addVisionUpdate(timestamp - getLatency(),
                // // getTarget(), Limelight.this);
                // updateDistanceToTarget();
                // }
                // }

                // if (mPeriodicIO.wantsChaseMode) {
                // aprilTagChasePeriodic();
                // }

                // send log data
                // SendLog();

                final double end = Timer.getFPGATimestamp();
                mPeriodicIO.dt = end - start;
            }

            @Override
            public void onStop(double timestamp) {
                stop();
                setLed(LedMode.OFF);
            }
        };
        mEnabledLooper.register(mLoop);
    }

    public synchronized boolean limelightOK() {
        return mPeriodicIO.has_comms;
    }

    public static class PeriodicIO {
        // INPUTS
        public double latency;
        public int givenLedMode;
        public int givenPipeline;
        public double xOffset;
        public double yOffset;
        public double area;
        public boolean has_comms;
        public boolean sees_target;

        public double limelightCameraPosex;
        public double limelightCameraPosey;
        public double limelightCameraPosez;
        public double limelightCameraPoseRoll;
        public double limelightCameraPosePith;
        public double limelightCameraPoseYaw;
        public double tagInView;
        public boolean wantsChaseMode;

        public double botPosex;
        public double botPosey;
        public double botPosez;
        public double botPoseRoll;
        public double botPosePitch;
        public double botPoseYaw;

        public double tanLineToSpeaker;

        public double dt;

        // OUTPUTS
        public int ledMode = 0; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
        public int stream = 0; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
        public double latencyTimestamp;
    }

    public synchronized List<TargetInfo> getTarget() {
        List<TargetInfo> targets = new ArrayList<TargetInfo>(); // getRawTargetInfos();
        targets.add(new TargetInfo(Math.tan(Math.toRadians(-mPeriodicIO.xOffset)),
                Math.tan(Math.toRadians(mPeriodicIO.yOffset))));
        if (hasTarget() && targets != null) {
            return targets;
        }

        return null;
    }

    private void initializeAprilTagChase() {
        xController = new ProfiledPIDController(2, 0, 0, Constants.VisionAlignConstants.X_CONSTRAINTS);
        yController = new ProfiledPIDController(1.5, 0, 0, Constants.VisionAlignConstants.Y_CONSTRAINTS); // 3 also
                                                                                                          // works for y
                                                                                                          // and x
        omegaController = new ProfiledPIDController(7, 0, 0, Constants.VisionAlignConstants.OMEGA_CONSTRAINTS);
        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(0.2));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        // lastTarget = null;
        robotPose = new Pose3d(
                mPeriodicIO.limelightCameraPosex,
                mPeriodicIO.limelightCameraPosey,
                0.0,
                new Rotation3d(0.0, 0.0, mPeriodicIO.limelightCameraPoseYaw * Math.PI / 180));
        // omegaController.reset(new TrapezoidProfile.State());
        // //robotPose.getRotation());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    public void isRedAlliance(boolean redAlliance) {
        isRedAlliance = redAlliance;
    }

    public double getTargetID() {
        return mPeriodicIO.tagInView;
    }

    private void aprilTagChasePeriodic() {
        // var limelightPose3d =
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new
        // double[6]);

        robotPose = new Pose3d(
                mPeriodicIO.limelightCameraPosex,
                mPeriodicIO.limelightCameraPosey,
                0.0,
                new Rotation3d(0.0, 0.0, mPeriodicIO.limelightCameraPoseYaw * Math.PI / 180)); // getRadians()));

        // if limelight has targets
        if ((mPeriodicIO.sees_target) && (mPeriodicIO.tagInView == Constants.VisionAlignConstants.TAG_TO_CHASE)) {
            // Find the tag we want to chase

            // Transform the tag's pose to set our goal
            Pose2d goalPose = robotPose.transformBy(Constants.VisionAlignConstants.TAG_TO_GOAL).toPose2d();

            SmartDashboard.putNumber("GoalPose x", goalPose.getX());
            SmartDashboard.putNumber("GoalPose y", goalPose.getY());
            SmartDashboard.putNumber("GoalPose rotation degrees", goalPose.getRotation().getDegrees());

            // Drive
            xController.setGoal(0);
            yController.setGoal(1.0);
            omegaController.setGoal(0);// goalPose.getRotation().getRadians()

            // Drive to the target
            double newXTarget = robotPose.getX();
            if ((newXTarget > -0.2) && (newXTarget < 0.2)) {
                newXTarget = 0.0;
            }
            double xSpeed = xController.calculate(newXTarget);
            if (xController.atGoal()) {
                xSpeed = 0;
            }

            double ySpeed = yController.calculate(robotPose.getY());
            if (yController.atGoal()) {
                ySpeed = 0;
            }

            double omegaSpeed = omegaController.calculate(mPeriodicIO.limelightCameraPoseYaw * Math.PI / 180);
            if (omegaController.atGoal()) {
                omegaSpeed = 0;
            }

            SmartDashboard.putNumber("drive call x", xSpeed);
            SmartDashboard.putNumber("drive call y", ySpeed);
            SmartDashboard.putNumber("drive call omega", omegaSpeed);

            // mSwerve.drive(new Translation2d(ySpeed,xSpeed), -omegaSpeed, false, false);
            mSwerve.feedTeleopSetpoint(new ChassisSpeeds(-ySpeed, -xSpeed, -0));
            // mSwerve.drive(new Translation2d(ySpeed,0), 0, true, false);
        } else {
            // No target has been visible
            mSwerve.feedTeleopSetpoint(new ChassisSpeeds(0, 0, 0));
        }

    }

    public Pose2d robotPose2d() {
        return new Pose2d(mPeriodicIO.limelightCameraPosex, mPeriodicIO.limelightCameraPosey,
                edu.wpi.first.math.geometry.Rotation2d.fromDegrees(mPeriodicIO.limelightCameraPoseYaw));
    }

    // public void updatePoseWithLimelight() {
    // if (mPeriodicIO.sees_target) {
    // mSwerve.addVisionMeasurement(
    // new Pose2d(new Translation2d(mPeriodicIO.botPosex, mPeriodicIO.botPosey),
    // edu.wpi.first.math.geometry.Rotation2d.fromDegrees(mPeriodicIO.botPoseYaw)),
    // mPeriodicIO.latencyTimestamp);

    // }
    // }

    public Pose2d robotPose2dInField() {
        return new Pose2d(mPeriodicIO.botPosex, mPeriodicIO.botPosey,
                edu.wpi.first.math.geometry.Rotation2d.fromDegrees(mPeriodicIO.botPoseYaw));
    }

    public void setWantChase(boolean chase) {
        mPeriodicIO.wantsChaseMode = chase;
    }

    // public double getLensHeight() {
    // return mConstants.kHeight;
    // }

    // public Rotation2d getHorizontalPlaneToLens() {
    // return mConstants.kHorizontalPlaneToLens;
    // }

    public Optional<Double> getLimelightDistanceToTarget() {
        return mDistanceToTarget;
    }

    public double getTanLineToSpeaker(){
        return mPeriodicIO.tanLineToSpeaker;
    }

    public void shootAgainstSubwooferSideAngle(boolean toggle){
        shootAgainstSubwooferSide = toggle;
    }

    public void setShootingFromPodium(boolean shootingFromPodium){
        if (shootFromPodium != shootingFromPodium){
            shootFromPodium = shootingFromPodium;
        }
    }

    public double getPivotShootingAngle() {
        //Right now we can use this to decide if we are shooting at the subwoofer or podium
        double pivAngle = Constants.PivotConstants.kShootAgainstSubwooferAngle+4.5;

        // if ((mPeriodicIO.tanLineToSpeaker>2)&&mPeriodicIO.sees_target){
        //     pivAngle = Constants.PivotConstants.kShootAgainstPodiumAngle;
        // }

        if (mPeriodicIO.sees_target){
            pivAngle = mRegression.getAngle(mPeriodicIO.tanLineToSpeaker);
        }

        if (shootAgainstSubwooferSide){
            pivAngle = Constants.PivotConstants.kShootAgainstSubwooferAngle+1.5;
        }

        if (shootFromPodium){
            pivAngle = Constants.PivotConstants.kShootAgainstPodiumAngle;
        }


        //double pivAngle = Math.pow((-0.105 * (mPeriodicIO.tanLineToSpeaker)) + 2.404, 4.87);
        // double pivAngle = 54.9-8.59*mPeriodicIO.tanLineToSpeaker +
        // 0.95*Math.pow(mPeriodicIO.tanLineToSpeaker, 2);
        SmartDashboard.putNumber("Pivot Limelight Generated angle", pivAngle);

        if (pivAngle > Constants.PivotConstants.kMaxAngle) {
            pivAngle = Constants.PivotConstants.kMaxAngle;
        } else if (pivAngle < Constants.PivotConstants.kMinAngle) {
            pivAngle = Constants.PivotConstants.kMinAngle;
        }

        return pivAngle;
    }

    public double getEndEffectorVelocityMaster(){
        double kShootVelocity = 0.925;
        // if (mPeriodicIO.sees_target && mPeriodicIO.tanLineToSpeaker<2){
        //     kShootVelocity = -6500;
        // }
        if (shootAgainstSubwooferSide){
            kShootVelocity = 0.805;

        }
        return kShootVelocity;
    }

    public double getEndEffectorVelocitySlave(){
        double kShootVelocity = 0.95;
        if (shootAgainstSubwooferSide){
            kShootVelocity = 0.83;

        }
        return kShootVelocity;
        
    }

    /** returns the degrees the robot should snap to */
    public double getTargetSnap() {
        double degreesToSnap = 180;
        if (mPeriodicIO.sees_target){
        //             if (mPeriodicIO.tanLineToSpeaker>1.8){
        //     degreesToSnap = 163;
        // }

        if ((mPeriodicIO.botPosey - 2.58) < 0) {
            degreesToSnap = 90
                    + (Math.atan(mPeriodicIO.botPosex / Math.abs(mPeriodicIO.botPosey - 2.58)) * (180 / Math.PI));
        } else {
            degreesToSnap = -90
                    - (Math.atan(mPeriodicIO.botPosex / Math.abs(mPeriodicIO.botPosey - 2.58)) * (180 / Math.PI));
        }
        
        //= Math.atan((mPeriodicIO.botPosey-2.6)/mPeriodicIO.botPosex);

        //-162
    }
        SmartDashboard.putNumber("degrees to snap to", degreesToSnap);
        return degreesToSnap;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        // final double latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0
        //         + (11.0 / 1000.0); // 90fps original latency calculation
        // final double latencyTimestamp = Timer.getFPGATimestamp() - (mNetworkTable.getEntry("cl").getDouble(0) / 1000.0)
        //         - (mNetworkTable
        //                 .getEntry("tl").getDouble(0) / 1000.0);
        // mPeriodicIO.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        // mPeriodicIO.givenPipeline = (int) mNetworkTable.getEntry("pipeline").getDouble(0);

        // if (latency == mPeriodicIO.latency) {
        //     mLatencyCounter++;
        // } else {
        //     mLatencyCounter = 0;
        // }

        // mPeriodicIO.latency = latency;
        // mPeriodicIO.latencyTimestamp = latencyTimestamp;
        // mPeriodicIO.has_comms = mLatencyCounter < 10;

        // mPeriodicIO.sees_target = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;

        // // double[] limelightCameraPose3d = tCameraPose.getDoubleArray(new double[]

        // if (isRedAlliance) {
        //     double[] robotPose3d = botpose_wpiblue.getDoubleArray(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
        //     mPeriodicIO.botPosex = robotPose3d[0];
        //     mPeriodicIO.botPosey = robotPose3d[1];
        //     mPeriodicIO.botPosez = robotPose3d[2];
        //     mPeriodicIO.botPoseRoll = robotPose3d[3];
        //     mPeriodicIO.botPosePitch = robotPose3d[4];
        //     mPeriodicIO.botPoseYaw = robotPose3d[5];
        // } else {
        //     double[] robotPose3d = botpose_wpired.getDoubleArray(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
        //     mPeriodicIO.botPosex = robotPose3d[0];
        //     mPeriodicIO.botPosey = robotPose3d[1];
        //     mPeriodicIO.botPosez = robotPose3d[2];
        //     mPeriodicIO.botPoseRoll = robotPose3d[3];
        //     mPeriodicIO.botPosePitch = robotPose3d[4];
        //     mPeriodicIO.botPoseYaw = robotPose3d[5];
        // }

        // mPeriodicIO.tagInView = tTargetID.getDouble(0.0); // .getDoubleArray(new double[] {0.0, 0.0, 0.0, 0.0, 0.0,
        //                                                   // 0.0})[0];
        // // {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}); // for some reason doing the array like this
        // // made it less buggy?
        // // mPeriodicIO.limelightCameraPosex = limelightCameraPose3d[0];
        // // mPeriodicIO.limelightCameraPosey = -limelightCameraPose3d[2];
        // // mPeriodicIO.limelightCameraPosez = -limelightCameraPose3d[1];
        // // mPeriodicIO.limelightCameraPoseRoll = limelightCameraPose3d[3];
        // // mPeriodicIO.limelightCameraPosePith = limelightCameraPose3d[4];
        // // mPeriodicIO.limelightCameraPoseYaw = limelightCameraPose3d[5];

        // // double[] robotPose3d = new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // mPeriodicIO.tanLineToSpeaker = Math
        //         .sqrt(mPeriodicIO.botPosex * mPeriodicIO.botPosex + Math.pow(mPeriodicIO.botPosey - 2.61, 2));
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.givenLedMode != mPeriodicIO.ledMode || mPeriodicIO.givenPipeline != mPeriodicIO.pipeline) {
            mOutputsHaveChanged = true;
        }
        if (mOutputsHaveChanged) {

            mNetworkTable.getEntry("ledMode").setNumber(mPeriodicIO.ledMode);
            mNetworkTable.getEntry("camMode").setNumber(mPeriodicIO.camMode);
            mNetworkTable.getEntry("pipeline").setNumber(mPeriodicIO.pipeline);
            mNetworkTable.getEntry("stream").setNumber(mPeriodicIO.stream);
            mNetworkTable.getEntry("snapshot").setNumber(mPeriodicIO.snapshot);

            mOutputsHaveChanged = false;
        }

    }

    @Override
    public void outputTelemetry() {
        // SmartDashboard.putBoolean("Limelight Ok (Has Comms)", mPeriodicIO.has_comms);
        // SmartDashboard.putNumber("Limelight Pipeline Latency (ms)", mPeriodicIO.latency);
        // SmartDashboard.putNumber("Limelight dt", mPeriodicIO.dt);

        // SmartDashboard.putBoolean("Limelight Has Target", mPeriodicIO.sees_target);
        // SmartDashboard.putNumber("Limelight tag ID In view", mPeriodicIO.tagInView);
        // // SmartDashboard.putNumber("Limelight Tx: ", mPeriodicIO.xOffset);
        // // SmartDashboard.putNumber("Limelight Ty: ", mPeriodicIO.yOffset);

        // // SmartDashboard.putNumber("Limelight Distance To Target",
        // // mDistanceToTarget.isPresent() ? mDistanceToTarget.get() : 0.0);

        // SmartDashboard.putNumber("Limelight x", mPeriodicIO.limelightCameraPosex);
        // SmartDashboard.putNumber("Limelight y", mPeriodicIO.limelightCameraPosey);
        // SmartDashboard.putNumber("Limelight z", mPeriodicIO.limelightCameraPosez);
        // SmartDashboard.putNumber("Limelight Yaw", mPeriodicIO.limelightCameraPoseYaw);

        // SmartDashboard.putNumber("limelight bot pose x", mPeriodicIO.botPosex);
        // SmartDashboard.putNumber("Limelight bot pose y", mPeriodicIO.botPosey);
        // SmartDashboard.putNumber("Limelight bot pose z", mPeriodicIO.botPosez);
        // SmartDashboard.putNumber("Limelight bot pose Yaw", mPeriodicIO.botPoseYaw);

        // SmartDashboard.putNumber("Tag In View", mPeriodicIO.tagInView);

        // SmartDashboard.putNumber("Limelight Tangent Line to Speaker", mPeriodicIO.tanLineToSpeaker);

        // SmartDashboard.putBoolean("WantChaseMode", mPeriodicIO.wantsChaseMode);
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }

    public synchronized void setLed(LedMode mode) {
        if (mode.ordinal() != mPeriodicIO.ledMode) {
            mPeriodicIO.ledMode = mode.ordinal();
            mOutputsHaveChanged = true;
        }
    }

    // public synchronized void setPipeline(int mode) {
    // if (mode != mPeriodicIO.pipeline) {
    // mPeriodicIO.pipeline = mode;

    // System.out.println(mPeriodicIO.pipeline + ", " + mode);
    // mOutputsHaveChanged = true;
    // }
    // }

    public synchronized void triggerOutputs() {
        mOutputsHaveChanged = true;
    }

    public synchronized int getPipeline() {
        return mPeriodicIO.pipeline;
    }

    public synchronized boolean hasTarget() {
        return mPeriodicIO.sees_target;
    }

    public synchronized boolean isOK() {
        return mPeriodicIO.has_comms;
    }

    public synchronized boolean isAimed() {
        if (hasTarget()) {
            return Util.epsilonEquals(mPeriodicIO.xOffset, 0.0, Constants.VisionAlignConstants.kEpsilon);
        } else {
            return false;
        }
    }

    public synchronized boolean isAutonomousAimed() {
        if (hasTarget()) {
            return Util.epsilonEquals(mPeriodicIO.xOffset, 0.0, 1.0);
        } else {
            return false;
        }
    }

    public double getLatency() {
        return mPeriodicIO.latency;
    }

    public double getDt() {
        return mPeriodicIO.dt;
    }

    public double[] getOffset() {
        return new double[] { mPeriodicIO.xOffset, mPeriodicIO.yOffset };
    }

}
