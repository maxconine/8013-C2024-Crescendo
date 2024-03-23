package com.team8013.frc2024.subsystems;

import com.team254.lib.util.Util;
import com.team8013.frc2024.Constants;
import com.team8013.frc2024.controlboard.ControlBoard;
import com.team8013.frc2024.loops.ILooper;
import com.team8013.frc2024.loops.Loop;
import com.team8013.lib.swerve.ChassisSpeeds;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.team8013.frc2024.regressions.ShooterRegression;

/**
 * Subsystem for interacting with the Limelight 3
 */
public class Limelight extends Subsystem {

    private static Limelight mInstance;
    private final Drive mSwerve = Drive.getInstance();
    private final ShooterRegression mRegression = new ShooterRegression();
    private ControlBoard mControlBoard = ControlBoard.getInstance();

    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController omegaController;
    private Pose2d robotPose;
    boolean isRedAlliance = true;
    //private boolean shootAgainstSubwooferSide = false;
    private boolean wantNoteChase = false;
    private boolean gottenNotePose = false;
    private boolean cantFindTargetOnInitialSnap = false;
    private Pose2d notePose = new Pose2d(0,0, new edu.wpi.first.math.geometry.Rotation2d(0));

    private int mLatencyCounter = 0;

    NetworkTable mNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");

    //NetworkTable mNetworkTableNoteVision = NetworkTableInstance.getDefault().getTable("limelight_Vision"); //TODO: for 2nd limelight

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private boolean mOutputsHaveChanged = true;
    private boolean shootFromPodium = false;

    private final NetworkTableEntry botpose_wpiblue = mNetworkTable.getEntry("botpose_wpiblue");
    private final NetworkTableEntry botpose_wpired = mNetworkTable.getEntry("botpose_wpired");
    private final NetworkTableEntry tTargetID = mNetworkTable.getEntry("tid");

    private Limelight() {
        //initializeNoteChase();
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

        public boolean noteInView;
        public double noteX;
        public double noteY;

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

    private void initializeNoteChase() { 
        xController = new ProfiledPIDController(2, 0, 0, Constants.VisionAlignConstants.X_CONSTRAINTS);
        yController = new ProfiledPIDController(2, 0, 0, Constants.VisionAlignConstants.Y_CONSTRAINTS); //3 also works
        omegaController = new ProfiledPIDController(7, 0, 0, Constants.VisionAlignConstants.OMEGA_CONSTRAINTS);
        xController.setTolerance(0.05); //change if having consistency issues was originally 0.2
        yController.setTolerance(0.05);
        omegaController.setTolerance(Units.degreesToRadians(0.2));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

    }

    public void wantNoteChase(boolean chase){
        if (wantNoteChase != chase){
            wantNoteChase = chase;
        }
    }

    private void noteChasePeriodic(){

        //set robot pose to odometry pose
        robotPose = mSwerve.getPose();

        //do I need to reset pid controllers every cycle even if I am using .calculate?
        // xController.reset(robotPose.getX());
        // yController.reset(robotPose.getY());
        // omegaController.reset(robotPose.getRotation().getRadians());

        // if limelight sees_note, update goal pose by the coordinates plus odometry
        //note: x positive going forwards and y positive going left
        if (mPeriodicIO.noteInView){
            //Pose2d notePose = new Pose2d(mPeriodicIO.noteX,mPeriodicIO.noteY,new edu.wpi.first.math.geometry.Rotation2d(0));
            


            Pose2d notePose = this.notePose;//new Pose2d(mPeriodicIO.noteX,mPeriodicIO.noteY, new edu.wpi.first.math.geometry.Rotation2d(0));

            Transform2d rotationTransform = new Transform2d(new Pose2d(0,0,new edu.wpi.first.math.geometry.Rotation2d(0)), notePose);

            // Transform the tag's pose to set our goal
            Pose2d goalPose = new Pose2d(robotPose.getX() + notePose.getY(),robotPose.getY() - notePose.getX(), rotationTransform.getRotation());

            SmartDashboard.putNumber("Goal Pose x", goalPose.getX());
            SmartDashboard.putNumber("Goal Pose y", goalPose.getY());
            SmartDashboard.putNumber("Goal Pose rotation degrees", goalPose.getRotation().getDegrees());

            // set pid controller goals
            xController.setGoal(goalPose.getX()); //previously all 0
            yController.setGoal(goalPose.getY());
            omegaController.setGoal(goalPose.getRotation().getRadians());// goalPose.getRotation().getRadians()

            // Drive to the target
            // double newXTarget = robotPose.getX();
            // if ((newXTarget > -0.2) && (newXTarget < 0.2)) {
            //     newXTarget = 0.0;
            // }

            double xSpeed = xController.calculate(robotPose.getX());
            if (xController.atGoal()) {
                xSpeed = 0;
            }

            double ySpeed = yController.calculate(robotPose.getY());
            if (yController.atGoal()) {
                ySpeed = 0;
            }

            double omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
            if (omegaController.atGoal()) {
                omegaSpeed = 0;
            }

            SmartDashboard.putNumber("drive call x", xSpeed);
            SmartDashboard.putNumber("drive call y", ySpeed);
            SmartDashboard.putNumber("drive call omega", omegaSpeed);

            // mSwerve.drive(new Translation2d(ySpeed,xSpeed), -omegaSpeed, false, false);
            mSwerve.feedTeleopSetpoint(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed));
            // mSwerve.drive(new Translation2d(ySpeed,0), 0, true, false);
        } else {
            // No target has been visible
            mSwerve.feedTeleopSetpoint(new ChassisSpeeds(0, 0, 0));
        }
    }
    

    public void isRedAlliance(boolean redAlliance) {
        isRedAlliance = !redAlliance; //opposite because it is flipped in normal code because I drew the trajectories on the wrong side
    }

    public double getTargetID() {
        return mPeriodicIO.tagInView;
    }


    public Pose2d robotPose2d() {
        return new Pose2d(mPeriodicIO.limelightCameraPosex, mPeriodicIO.limelightCameraPosey,
                edu.wpi.first.math.geometry.Rotation2d.fromDegrees(mPeriodicIO.limelightCameraPoseYaw));
    }

    // public void updatePoseWithLimelight() {
    // if (mPeriodicIO.sees_target) {
    // mSwerve.addVisionMeasurement(
    // new Pose2d(new Translation2d(mPeriodicIO.botPosex, mPeriodicIO.botPosey),),
    // mPeriodicIO.latencyTimestamp);
    // }
    // }

    public Pose2d robotPose2dInField() {
        return new Pose2d(mPeriodicIO.botPosex, mPeriodicIO.botPosey,
                edu.wpi.first.math.geometry.Rotation2d.fromDegrees(mPeriodicIO.botPoseYaw));
    }

    public double getTanLineToSpeaker(){
        return mPeriodicIO.tanLineToSpeaker;
    }

    // public void shootAgainstSubwooferSideAngle(boolean toggle){
    //     shootAgainstSubwooferSide = toggle;
    // }

    public void setShootingFromPodium(boolean shootingFromPodium){
        if (shootFromPodium != shootingFromPodium){
            shootFromPodium = shootingFromPodium;
        }
    }

    public double getPivotShootingAngle() {
        //Right now we can use this to decide if we are shooting at the subwoofer or podium
        double pivAngle = Constants.PivotConstants.kShootAgainstSubwooferAngle;

        // if ((mPeriodicIO.tanLineToSpeaker>2)&&mPeriodicIO.sees_target){
        //     pivAngle = Constants.PivotConstants.kShootAgainstPodiumAngle;
        // }

        if (mPeriodicIO.sees_target && mControlBoard.snapToTarget()){
            pivAngle = mRegression.getAngle(mPeriodicIO.tanLineToSpeaker);
        }

        // if (shootAgainstSubwooferSide){
        //     pivAngle = Constants.PivotConstants.kShootAgainstSubwooferAngle+1.5;
        // }

        // if (shootFromPodium){
        //     pivAngle = Constants.PivotConstants.kShootAgainstPodiumAngle;
        // }

        SmartDashboard.putNumber("Pivot Limelight Generated angle", pivAngle);

        pivAngle = Util.limit(pivAngle, Constants.PivotConstants.kMinAngle, Constants.PivotConstants.kMaxAngle);

        return pivAngle;
    }

    public double getEndEffectorShootingVelocity(){
        double vel = Constants.EndEffectorConstants.kSubwooferRPM;
        if (mControlBoard.snapToTarget()){
            vel = mRegression.getRPM(mPeriodicIO.tanLineToSpeaker);
        }
        vel = Util.limit(vel, Constants.EndEffectorConstants.kSubwooferRPM,6600);
        SmartDashboard.putNumber("Limelight Generated RPM", vel);
        return vel;
    }

    /** returns the degrees the robot should snap to in order to shoot in the speaker*/
    public double getTargetSnap() {
        double degreesToSnap = 180;
        cantFindTargetOnInitialSnap = true;

        //instead of dead reckoning, this is using odometry
        // Transform2d transformOdometry = new Transform2d(new Pose2d(mSwerve.getPoseX(), mSwerve.getPoseY(), new edu.wpi.first.math.geometry.Rotation2d(0)),
        // speakerPoseOnField());
        // Rotation2d rot = new Rotation2d(transformOdometry.getX(), transformOdometry.getY(), true); //this might also work, worth a try
        // double degreesToSnap = transformOdometry.getRotation().getDegrees();

        if (mPeriodicIO.sees_target){
        //             if (mPeriodicIO.tanLineToSpeaker>1.8){
        //     degreesToSnap = 163;
        // }

        // if ((mPeriodicIO.botPosey - 2.58) < 0) { //this equation needs to be worked out
        //     degreesToSnap = 90
        //             + (Math.atan(mPeriodicIO.botPosex / Math.abs(mPeriodicIO.botPosey - 2.58)) * (180 / Math.PI));

        
        // } else {
        //     degreesToSnap = -90
        //             - (Math.atan(mPeriodicIO.botPosex / Math.abs(mPeriodicIO.botPosey - 2.58)) * (180 / Math.PI));
        // }
            cantFindTargetOnInitialSnap = false;

            Transform2d transform = new Transform2d(new Pose2d(mPeriodicIO.botPosex, mPeriodicIO.botPosey, new edu.wpi.first.math.geometry.Rotation2d(0)),
            speakerPoseOnField());

            degreesToSnap = transform.getRotation().getDegrees(); //if not facing the right direction try making the goal pose rotation value 180

            //if the above doesn't work try atan2
            //degreesToSnap = Math.atan2(mPeriodicIO.botPosex, mPeriodicIO.botPosey-2.58); //angle looking from the speaker to the robot -(really weird)
        }
        SmartDashboard.putNumber("degrees to snap to", degreesToSnap);
        return degreesToSnap;
    }

    private double doTanLineToSpeakerMath(){ //make sure to 
        if (mPeriodicIO.sees_target){
            Transform2d transform = new Transform2d(limelightBotPose2d(),speakerPoseOnField());
            return transform.getTranslation().getNorm();
        }
        else{ //should not be needed
            // Transform2d transform = new Transform2d(mSwerve.getPose(),speakerPoseOnField());
            // return transform.getTranslation().getNorm();
            return 0.0;
        }
    }

    public Pose2d limelightBotPose2d(){
        return new Pose2d(mPeriodicIO.botPosey,mPeriodicIO.botPosex, new edu.wpi.first.math.geometry.Rotation2d(mPeriodicIO.botPoseYaw * (Math.PI/180)));
    }
    public Pose2d speakerPoseOnField(){
        return new Pose2d(0,2.58,new edu.wpi.first.math.geometry.Rotation2d(0));
    }

    public boolean cantFindTargetOnInitialSnap(){
        return cantFindTargetOnInitialSnap;
    }

    public boolean isDoneWithNotePickup(){//zero the odometry with 'known' location?
        return (xController.atGoal() && yController.atGoal()); //add theta?
    }



    @Override
    public synchronized void readPeriodicInputs() {
        final double latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0
                + (11.0 / 1000.0); // 90fps original latency calculation
        // final double latencyTimestamp = Timer.getFPGATimestamp() - (mNetworkTable.getEntry("cl").getDouble(0) / 1000.0)
        //         - (mNetworkTable
        //                 .getEntry("tl").getDouble(0) / 1000.0);
        mPeriodicIO.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        mPeriodicIO.givenPipeline = (int) mNetworkTable.getEntry("pipeline").getDouble(0);

        if (latency == mPeriodicIO.latency) {
            mLatencyCounter++;
        } else {
            mLatencyCounter = 0;
        }

        mPeriodicIO.latency = latency;
        // mPeriodicIO.latencyTimestamp = latencyTimestamp;
        mPeriodicIO.has_comms = mLatencyCounter < 10;

        mPeriodicIO.sees_target = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;

        if (isRedAlliance) {
            double[] robotPose3d = botpose_wpired.getDoubleArray(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
            mPeriodicIO.botPosex = robotPose3d[0];
            mPeriodicIO.botPosey = robotPose3d[1];
            mPeriodicIO.botPosez = robotPose3d[2];
            mPeriodicIO.botPoseRoll = robotPose3d[3];
            mPeriodicIO.botPosePitch = robotPose3d[4];
            mPeriodicIO.botPoseYaw = robotPose3d[5];
        } else {
            double[] robotPose3d = botpose_wpiblue.getDoubleArray(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
            mPeriodicIO.botPosex = robotPose3d[0];
            mPeriodicIO.botPosey = robotPose3d[1];
            mPeriodicIO.botPosez = robotPose3d[2];
            mPeriodicIO.botPoseRoll = robotPose3d[3];
            mPeriodicIO.botPosePitch = robotPose3d[4];
            mPeriodicIO.botPoseYaw = robotPose3d[5];
        }

        mPeriodicIO.tagInView = tTargetID.getDouble(0.0);

        // mPeriodicIO.tanLineToSpeaker = Math
        //         .sqrt(mPeriodicIO.botPosex * mPeriodicIO.botPosex + Math.pow(mPeriodicIO.botPosey - 2.61, 2));
        mPeriodicIO.tanLineToSpeaker = doTanLineToSpeakerMath();

        //add note vision updates
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

        if (wantNoteChase){
            noteChasePeriodic();
        }

        //if robot is not moving and targets in view, zero the odometry??

        //gets the note pose that is used during the note chase once so that there is no camera delay while chasing
        if (!gottenNotePose && wantNoteChase){
            notePose = new Pose2d(mPeriodicIO.noteX,mPeriodicIO.noteY, new edu.wpi.first.math.geometry.Rotation2d(0));
            gottenNotePose = true;
        }
        if (!wantNoteChase){
            gottenNotePose = false;
        }

    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Limelight Ok (Has Comms)", mPeriodicIO.has_comms);
        SmartDashboard.putNumber("Limelight Pipeline Latency (ms)", mPeriodicIO.latency);
        SmartDashboard.putNumber("Limelight dt", mPeriodicIO.dt);

        SmartDashboard.putBoolean("Limelight Has Target", mPeriodicIO.sees_target);
        SmartDashboard.putNumber("Limelight tag ID In view", mPeriodicIO.tagInView);

        SmartDashboard.putNumber("limelight bot pose x", mPeriodicIO.botPosex);
        SmartDashboard.putNumber("Limelight bot pose y", mPeriodicIO.botPosey);
        SmartDashboard.putNumber("Limelight bot pose z", mPeriodicIO.botPosez);
        SmartDashboard.putNumber("Limelight bot pose Yaw", mPeriodicIO.botPoseYaw);

        SmartDashboard.putNumber("Tag In View", mPeriodicIO.tagInView);

        SmartDashboard.putNumber("Limelight Tangent Line to Speaker", mPeriodicIO.tanLineToSpeaker);

        // SmartDashboard.putBoolean("WantChaseMode", mPeriodicIO.wantsChaseMode);

        //add note vision updates
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

    //can fill these in later, checks if the robot is aimed at the tag, need to update xOffset
    // public synchronized boolean isAimed() {
    //     if (hasTarget()) {
    //         return Util.epsilonEquals(mPeriodicIO.xOffset, 0.0, Constants.VisionAlignConstants.kEpsilon);
    //     } else {
    //         return false;
    //     }
    // }

    // public synchronized boolean isAutonomousAimed() {
    //     if (hasTarget()) {
    //         return Util.epsilonEquals(mPeriodicIO.xOffset, 0.0, 1.0);
    //     } else {
    //         return false;
    //     }
    // }

    public double getLatency() {
        return mPeriodicIO.latency;
    }

    // public double getDt() {
    //     return mPeriodicIO.dt;
    // }

    // public double[] getOffset() {
    //     return new double[] { mPeriodicIO.xOffset, mPeriodicIO.yOffset };
    // }

}
