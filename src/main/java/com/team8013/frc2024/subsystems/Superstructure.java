package com.team8013.frc2024.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.team8013.frc2024.Constants;
import com.team8013.frc2024.FieldLayout;
import com.team8013.frc2024.controlboard.ControlBoard;
import com.team8013.frc2024.loops.CrashTracker;
import com.team8013.frc2024.loops.ILooper;
import com.team8013.frc2024.loops.Loop;
import com.team8013.frc2024.subsystems.EndEffector.State;
import com.team8013.frc2024.subsystems.Shooter.ControlState;
import com.team8013.lib.logger.Log;
import com.team8013.lib.requests.Request;
import com.team8013.lib.requests.SequentialRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem {

    private Elevator mElevator = Elevator.getInstance();
    private Wrist mWrist = Wrist.getInstance();
    private EndEffector mEndEffector = EndEffector.getInstance();
    private Drive mDrive = Drive.getInstance();
    private Limelight mLimelight = Limelight.getInstance();
    private ControlBoard mControlBoard = ControlBoard.getInstance();
    private Pivot mPivot = Pivot.getInstance();
    private Shooter mShooter = Shooter.getInstance();

    private Request activeRequest = null;
    private ArrayList<Request> queuedRequests = new ArrayList<>(0);
    private boolean hasNewRequest = false;
    private boolean allRequestsComplete = false;

    private double pivotManualPosition = mPivot.getPivotAngleDeg();
    private double elevatorManualPosition = mElevator.getElevatorUnits();
    private double wristManualPosition = mWrist.getWristAngleDeg();
    private SuperstructureState mSuperstructureState;

    private boolean manualControlMode;
    private boolean outtake;
    private boolean wantsManualIntake;
    private boolean outtakingTimerStarted = false;
    private int climbingTracker = -1;
    private boolean climbModeStage2 = false;
    private boolean climbModeStage3 = false;
    private boolean bringElevatorIntoLoad = false;
    private Timer outtakingTimer = new Timer();

    public boolean requestsCompleted() {
        return allRequestsComplete;
    }

    // private ScoringLocation target_location = new ScoringLocation();

    /* Singleton Instance */
    private static Superstructure mInstance;

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    };

    public void request(Request r) {
        setActiveRequest(r);
        clearRequestQueue();
    }

    private void setActiveRequest(Request request) {
        activeRequest = request;
        hasNewRequest = true;
        allRequestsComplete = false;
    }

    private void clearRequestQueue() {
        queuedRequests.clear();
    }

    private void setRequestQueue(List<Request> requests) {
        clearRequestQueue();
        for (Request req : requests) {
            queuedRequests.add(req);
        }
    }

    private void setRequestQueue(Request activeRequest, ArrayList<Request> requests) {
        request(activeRequest);
        setRequestQueue(requests);
    }

    private void addRequestToQueue(Request req) {
        queuedRequests.add(req);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                clearRequestQueue();
                // stowState();
            }

            @Override
            public void onLoop(double timestamp) {
                try {
                    if (hasNewRequest && activeRequest != null) {
                        activeRequest.act();
                        hasNewRequest = false;
                    }

                    if (activeRequest == null) {
                        if (queuedRequests.isEmpty()) {
                            allRequestsComplete = true;
                        } else {
                            request(queuedRequests.remove(0));
                        }
                    } else if (activeRequest.isFinished()) {
                        activeRequest = null;
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }

                SmartDashboard.putBoolean("All reqs complete", allRequestsComplete);

                // ScoringLocation other = ControlBoard.getInstance().updateScoringLocation();
                // if (!other.equals(target_location)) {
                // target_location = other;
                // System.out.println("New scoring location!");
                // }
                // wantsCube = (target_location.column == 2);
                // SmartDashboard.putBoolean("Wants cube", wantsCube);

                // if (!is_climbing) {
                // updateLEDs();
                // }
            }

            @Override
            public void onStop(double timestamp) {
                clearRequestQueue();
            }
        });
    }

    // public ScoringLocation getScoringLocation() {
    // return target_location;
    // }

    // public void setScoringLocation(ScoringLocation scoringLocation) {
    // target_location = scoringLocation;
    // }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    Timer flash_timeout = new Timer();

    /*
     * Odometry: y+ left
     * x+ foreward
     * 
     * Bot Pose (blue alliance)
     * y+ left
     * x+ foreward
     * 
     * 
     * Steps for trajectory to tag
     * get robot location on field and update odometry (may need to not update the
     * gyro?)
     * use known tag location of ID 1 with x cord moved 1 meter up as the other
     * point2d
     * depending on the side of the field make the rotation2d so that the bot is
     * facing towards you in the trajectory (either 0 or 180)
     * generate a trajectory from the DriveMotionPlanner and send it to
     * mDrive.setTrajectory()
     * see if it works..
     * 
     * 
     */

    public void tagTrajectory(int tagToChase, boolean chaseNearest) {
        Pose2d robotPose = mLimelight.robotPose2dInField();
        Pose3d IDPose3d = FieldLayout.aprilTags.get(tagToChase);

        if ((robotPose.getX() != 0) && (robotPose.getY() != 0)) {
            mDrive.resetOdometry(robotPose);
        } else {
            robotPose = mDrive.getPose();
        }

        if (chaseNearest) {
            double tID = mLimelight.getTargetID();
            if (tID == -1) {
                return;
            }

            IDPose3d = FieldLayout.aprilTags.get((int) tID);
        }

        Pose2d IDPose2d = new Pose2d(IDPose3d.getX() - 1.5, IDPose3d.getY(),
                Rotation2d.fromRadians(IDPose3d.getRotation().getAngle() + Math.PI / 2)); //

        double angle = calculateAngle(robotPose, IDPose2d);

        SmartDashboard.putNumber("Angle 1", angle);

        Pose2d pose1 = new Pose2d(robotPose.getTranslation(),
                Rotation2d.fromDegrees(angle));
        Pose2d pose2 = new Pose2d(IDPose2d.getTranslation(),
                Rotation2d.fromDegrees(angle));

        if (!arePointsSeparated(pose1, pose2)) {
            pose1 = new Pose2d(robotPose.getTranslation(), new Rotation2d());
            pose2 = new Pose2d(IDPose2d.getTranslation(), new Rotation2d());
            SmartDashboard.putBoolean("rotations set to default", true);
        } else {
            SmartDashboard.putBoolean("rotations set to default", false);
        }

        SmartDashboard.putNumber("Goal Trajectory Start Location X", robotPose.getX());
        SmartDashboard.putNumber("Goal Trajectory Start Location Y", robotPose.getY());
        SmartDashboard.putNumber("Goal Trajectory Location X", IDPose3d.getX() - 1.5);
        SmartDashboard.putNumber("Goal Trajectory Location Y", IDPose3d.getY());

        Trajectory traj = generateTrajectory(Constants.VisionAlignConstants.TAG_TRAJECTORY_CONFIG,
                pose1, pose2);
        mDrive.setTrajectory(traj, Rotation2d.fromDegrees(0));

    }

    public Trajectory generateTrajectory(TrajectoryConfig config, Pose2d... poses) {

        ArrayList<Translation2d> interiorPoints = new ArrayList<Translation2d>();
        for (int i = 1; i < poses.length - 1; i++) {
            interiorPoints.add(poses[i].getTranslation());
        }
        try {
            return TrajectoryGenerator.generateTrajectory(poses[0], interiorPoints, poses[poses.length - 1], config);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

    }

    public double calculateAngle(Pose2d point1, Pose2d point2) {
        double x1 = point1.getX();
        double y1 = point1.getY();
        double x2 = point2.getX();
        double y2 = point2.getY();

        double angle = Math.atan2(y2 - y1, x2 - x1);
        double angleDegrees = Math.toDegrees(angle);

        if (angleDegrees < 0) {
            angleDegrees += 360;
        }

        return angleDegrees;
    }

    public boolean arePointsSeparated(Pose2d point1, Pose2d point2) {
        double xDistance = Math.abs(point1.getX() - point2.getX());
        double yDistance = Math.abs(point1.getY() - point2.getY());

        return xDistance > 0.5 || yDistance > 0.5;
    }

    public boolean isFinishedWithTagTrajectory() {
        if (mDrive.isDoneWithTrajectory()) {
            if (mLimelight.hasTarget() && resetOdometryToLimelight(mDrive.getPose(), mLimelight.robotPose2dInField())) {
                tagTrajectory(mControlBoard.tagToChase(), mControlBoard.chaseNearest());
                return false;
            }

            mDrive.stopModules();
            if (mLimelight.isRedAlliance) {
                mDrive.zeroGyro(180);
            } else {
                mDrive.zeroGyro(0);
            }
            return true;
        }
        return false;
    }

    public boolean resetOdometryToLimelight(Pose2d pose1, Pose2d pose2) {
        if (Math.abs((Math.abs(pose1.getX())) - Math.abs(pose2.getX())) > Constants.VisionAlignConstants.POSITION_OFF) {
            return true;
        } else if (Math
                .abs((Math.abs(pose1.getY())) - Math.abs(pose2.getY())) > Constants.VisionAlignConstants.POSITION_OFF) {
            return true;
        } else
            return false;
    }

    public void controlPivotManually(double demand) {
        // double position = mPivot.getPivotAngleDeg();
        if (demand > 0.3) {
            pivotManualPosition += 0.2;
        } else if (demand < -0.3) { // &&(pivotManualPosition>Constants.PivotConstants.kMinAngle)
            pivotManualPosition = pivotManualPosition - 0.2;
        }

        mPivot.setSetpointMotionMagic(pivotManualPosition);
    }

    public void controlElevatorManually(double demand) {
        if ((demand > 0.2)) {
            elevatorManualPosition += 0.001;
        } else if ((demand < -0.2)) { // &&(pivotManualPosition>Constants.PivotConstants.kMinAngle)
            elevatorManualPosition += -0.001;
        }

        mElevator.setSetpointMotionMagic(elevatorManualPosition);
    }

    public void controlWristManually(double demand) {
        if (demand > 0) {// &&(position<Constants.WristConstants.kMaxPosition)){
            wristManualPosition += 0.2;
        } else if (demand < 0) {// &&(position<Constants.WristConstants.kMinPosition)){
            wristManualPosition = wristManualPosition - 0.2;
        }
        // SmartDashboard.putNumber("wristManualPosition", wristManualPosition);
        mWrist.setSetpointMotionMagic(wristManualPosition);
    }

    public void intake(boolean intake) {
        if (intake) {
            mEndEffector.setState(State.INTAKING);
        } else {
            mEndEffector.setState(State.IDLE);
        }
    }

    public void outtake(boolean outtake) {
        if (outtake) {
            mEndEffector.setState(State.OUTTAKING);
        } else {
            mEndEffector.setState(State.IDLE);
        }
    }

    public enum SuperstructureState {
        INTAKING_GROUND,
        INTAKING_SOURCE,
        SCORE_AMP,
        TRANSFER_TO_SHOOTER,
        SHOOT,
        STOW,
        CLIMB
    }

    public void setSuperstuctureIntakingGround() {
        if (mSuperstructureState != SuperstructureState.INTAKING_GROUND) {
            mSuperstructureState = SuperstructureState.INTAKING_GROUND;

            mWrist.setSetpointMotionMagic(265);
            // flips the wrist down immediatly, then elevator waits for it to get into
            // position before extending

        }
    }

    public void setSuperstuctureIntakingSource() {
        if (mSuperstructureState != SuperstructureState.INTAKING_SOURCE) {
            mSuperstructureState = SuperstructureState.INTAKING_SOURCE;
        }
    }

    public void setSuperstuctureScoreAmp() {
        if (mSuperstructureState != SuperstructureState.SCORE_AMP) {
            mSuperstructureState = SuperstructureState.SCORE_AMP;
        }
    }

    public void setSuperstuctureTransferToShooter() {
        if (mSuperstructureState != SuperstructureState.TRANSFER_TO_SHOOTER) {
            mSuperstructureState = SuperstructureState.TRANSFER_TO_SHOOTER;
            mWrist.setSetpointMotionMagic(Constants.WristConstants.kloadShooterAngle);
            mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kloadShooterInitialHeight);
            mPivot.setSetpointMotionMagic(Constants.PivotConstants.kShootLoadAngle);
        }
    }

    public void setSuperstuctureShoot() {
        if (mSuperstructureState != SuperstructureState.SHOOT) {
            mSuperstructureState = SuperstructureState.SHOOT;
        }
    }

    public void setSuperstuctureStow() {
        if (mSuperstructureState != SuperstructureState.STOW) {
            mSuperstructureState = SuperstructureState.STOW;
            mPivot.setSetpointMotionMagic(12);
        }
    }

    public void setManualControlMode(boolean isManualControl) {
        if (manualControlMode != isManualControl) {
            manualControlMode = isManualControl;
        }
    }

    public void setClimbMode() {
        if (mSuperstructureState != SuperstructureState.CLIMB) {
            mSuperstructureState = SuperstructureState.CLIMB;

            mElevator.setMotorConfig(Constants.ElevatorConstants.elevatorSlowMotorConfig());
            mPivot.setMotorConfig(Constants.PivotConstants.pivotSlowMotorConfig());

            mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kClimbHeight);
            mPivot.setSetpointMotionMagic(Constants.PivotConstants.kClimbAngle);
            mWrist.setSetpointMotionMagic(Constants.WristConstants.kClimbAngle);
            climbingTracker = -1;
        }
    }

    public void setClimbModeStage2() {
        if (mSuperstructureState == SuperstructureState.CLIMB) {
            climbModeStage2 = true;
        }
    }

    public void setClimbModeStage3() {
        if (mSuperstructureState == SuperstructureState.CLIMB) {
            climbModeStage3 = true;
        }
    }

    public boolean inClimbMode() {
        return mSuperstructureState == SuperstructureState.CLIMB;
    }

    public void setWantOuttake(boolean _outtake) {
        if (outtake != _outtake) {
            outtake = _outtake;
        }
    }

    public void setWantIntake(boolean _intake) {
        if (wantsManualIntake != _intake) {
            wantsManualIntake = _intake;
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (manualControlMode) {
            // do manual control things
        } else {
            if (mSuperstructureState == SuperstructureState.STOW) {
                mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kStowHeight);

                mWrist.setSetpointMotionMagic(getPositionsGroundIntakeIn(mElevator.getElevatorUnits())[0]);
                mPivot.setSetpointMotionMagic(getPositionsGroundIntakeIn(mElevator.getElevatorUnits())[1]);

                if (mWrist.getWristAngleDeg() < 320) {
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kStowAngle);
                }
                // mWrist.setSetpointMotionMagic(Constants.WristConstants.kStowAngle);
                // mPivot.setSetpointMotionMagic(Constants.PivotConstants.kStowAngle);

            } else if (mSuperstructureState == SuperstructureState.SHOOT) {
                mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kShootHeight);
                mWrist.setSetpointMotionMagic(Constants.WristConstants.kShootAngle);
                mPivot.setSetpointMotionMagic(Constants.PivotConstants.kShootAgainstSubwooferAngle); // in the future
                                                                                                     // this will adjust
                                                                                                     // based on
                                                                                                     // position from
                                                                                                     // goal
            } else if (mSuperstructureState == SuperstructureState.TRANSFER_TO_SHOOTER) {

                // does stuff up above

                // once wrist & elevator in position

                if (bringElevatorIntoLoad
                        && mElevator.getElevatorUnits() < Constants.ElevatorConstants.kloadShooterFinalHeight
                                + Constants.ElevatorConstants.kPositionError) {
                    // once elevator down,
                    bringElevatorIntoLoad = false;
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kloadShooterInitialHeight);
                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kShootAgainstSubwooferAngle);
                    mEndEffector.setEndEffectorVelocity(2000);

                    // if (want shoot) then shoot
                } else if (bringElevatorIntoLoad) {
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kloadShooterFinalHeight);
                    // mEndEffector.setState(State.OUTTAKING);
                    // mShooter.setOpenLoopDemand(-0.1);
                } else if (mWrist.getWristAngleDeg() < Constants.WristConstants.kloadShooterAngle + 10
                        && mElevator.getElevatorUnits() > Constants.ElevatorConstants.kloadShooterInitialHeight
                                - Constants.ElevatorConstants.kPositionError) {
                    bringElevatorIntoLoad = true;
                }

                // once loaded, bring elevator back out to shoot

                // then outtake
            } else if (mSuperstructureState == SuperstructureState.SCORE_AMP) {
                mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kAmpScoreHeight);
                mWrist.setSetpointMotionMagic(Constants.WristConstants.kAmpScoreAngle);
                mPivot.setSetpointMotionMagic(Constants.PivotConstants.kAmpScoreAngle);

            } else if (mSuperstructureState == SuperstructureState.INTAKING_GROUND) {

                if (mWrist.getWristAngleDeg() > 250) {
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kFloorIntakeHeight);

                    mWrist.setSetpointMotionMagic(getPositionsGroundIntakeOut(mElevator.getElevatorUnits())[0]);
                    mPivot.setSetpointMotionMagic(getPositionsGroundIntakeOut(mElevator.getElevatorUnits())[1]);
                }

                if (!mEndEffector.hasGamePiece() && mWrist.getWristAngleDeg() > 350) {
                    mEndEffector.setState(State.INTAKING);
                } else if (mEndEffector.hasGamePiece()) {
                    mEndEffector.setState(State.IDLE);
                    // Once game piece aquired, then stow
                    mSuperstructureState = SuperstructureState.STOW;
                }

            } else if (mSuperstructureState == SuperstructureState.INTAKING_SOURCE) {
                mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kSourceIntakeHeight);
                mPivot.setSetpointMotionMagic(Constants.PivotConstants.kSourceIntakeAngle);

                if (mPivot.getPivotAngleDeg() > Constants.PivotConstants.kSourceIntakeAngle - 55) {
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kSourceIntakeAngle);
                }

            } else if (mSuperstructureState == SuperstructureState.CLIMB) {
                /* CLIMB STATE YAY */
                // setup climb in init state change

                /*
                 * Climbing Tracker
                 * 0: climb Setup -- ready to hook on
                 * 1: halfway pulled onto chain, ready for pivot
                 * 2: hooked onto chain, ready for pivot to go up and elevator to extend
                 * 3: unhooked elevator from chain, ready for user input to raise up and score
                 * in trap
                 * 4: elevator and pivot extended out, ready for quick pivot
                 * 5:
                 */

                // Stage 1: set up climb
                if ((mElevator.getElevatorUnits() >= Constants.ElevatorConstants.kClimbHeight
                        - Constants.ElevatorConstants.kPositionError
                        && mPivot.getPivotAngleDeg() > Constants.PivotConstants.kClimbAngle
                                - Constants.PivotConstants.kPositionError)
                        && (!climbModeStage2) && (!climbModeStage3)) {

                    // climbSetup = true;
                    climbingTracker = 0;
                    // SmartDashboard.putBoolean("Climber In Position", true);
                    // SmartDashboard.putNumber("Climbing Stage", 1);

                }

                // Stage 2: once climb set up, wait for user to press button to pull down to
                // chain
                if (climbModeStage2 && climbingTracker == 0) {
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kPullOntoChainHeight);
                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kPullOntoChainAngle1);
                    climbingTracker = 1;
                }

                if (climbingTracker == 1
                        && (mElevator.getElevatorUnits() < Constants.ElevatorConstants.kPullOntoChainHeight
                                + Constants.ElevatorConstants.kPositionError)) {
                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kPullOntoChainAngle2);
                    climbingTracker = 2;
                }

                if (climbingTracker == 2 && (mPivot.getPivotAngleDeg() < Constants.PivotConstants.kPullOntoChainAngle2
                        + Constants.PivotConstants.kPositionError)) {
                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kExtendOffChainAngle1);
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kExtendOffChain1);
                    climbingTracker = 3;
                }

                // Stage 3: wait for user to press button to extend up to trap
                if (climbingTracker == 3 && climbModeStage3) {
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kExtendOffChain2);
                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kExtendOffChainAngle2);
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kClimbScoreInTrapAngle);
                    climbingTracker = 4;
                }

                if ((climbingTracker == 4)
                        && (mElevator.getElevatorUnits() > Constants.ElevatorConstants.kExtendOffChain2
                                - Constants.ElevatorConstants.kPositionError)
                        && mPivot.getPivotAngleDeg() > Constants.PivotConstants.kExtendOffChainAngle2
                                - Constants.PivotConstants.kPositionError) {
                    climbingTracker = 5;
                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kExtendToScoreTrapAngle); // fast
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kExtendToScoreTrapHeight);
                }

                if ((climbingTracker == 5)
                        && (mPivot.getPivotAngleDeg() > Constants.PivotConstants.kExtendToScoreTrapAngle
                                - Constants.PivotConstants.kPositionError)) {
                    // on the trap wall pressed against it, maybe shake the pivot to wedge the end
                    // effector in
                }

            }
        }

        if (outtake)

        {
            mEndEffector.setState(State.OUTTAKING);
        } else if (wantsManualIntake) {
            mEndEffector.setState(State.INTAKING);
        } else { // if (mSuperstructureState != SuperstructureState.INTAKING_GROUND &&
            // mSuperstructureState != SuperstructureState.INTAKING_SOURCE){
            mEndEffector.setState(State.IDLE);

            // SmartDashboard.putString("Superstructure State",
            // mSuperstructureState.toString());

        }
    }

    private double[] getPositionsGroundIntakeOut(double elevatorPosition) {
        /*
         * { 0.004, 280,0.0},
         * {0.03,288,0.0},
         * {0.052,293,0.0},
         * {0.075,300,1},
         * {0.1,306,2},
         * {0.125, 313,4},
         * {0.15,321,5.2},
         * {0.175,327,5.3},
         * {0.2,335,5.5},
         * {0.215,338,5.1},
         * {0.23,343,5.2},
         * {0.25,347,3.5},
         * {0.275,359.5,1.6}
         * 
         */
        int index = 0;
        for (int i = 0; i < Constants.ElevatorConstants.groundIntakeWristPositions.length; i++) {
            if (Constants.ElevatorConstants.groundIntakeWristPositions[i][0] < elevatorPosition) {
                index = i;
            }
        }
        return new double[] { Constants.ElevatorConstants.groundIntakeWristPositions[index][1],
                Constants.ElevatorConstants.groundIntakeWristPositions[index][2] };
    }

    private double[] getPositionsGroundIntakeIn(double elevatorPosition) {

        int index = 0;
        for (int i = Constants.ElevatorConstants.groundIntakeWristPositions.length - 1; i >= 0; i--) {
            if (Constants.ElevatorConstants.groundIntakeWristPositions[i][0] > elevatorPosition) {
                index = i;
            }
        }
        return new double[] { Constants.ElevatorConstants.groundIntakeWristPositions[index][1] - 10,
                Constants.ElevatorConstants.groundIntakeWristPositions[index][2] + 1 };
    }

    // public void updateLEDs() {
    // if (mLEDs.getUsingSmartdash()) {
    // return;
    // }
    // State state = State.OFF;

    // if (mEndEffector.hasGamePiece()) {
    // if (flash_timeout.get() > 1.5) {
    // if (target_location.level == 1) {
    // state = State.SOLID_BLUE;
    // } else if (target_location.level == 2) {
    // state = State.SOLID_PINK;
    // } else {
    // state = State.SOLID_GREEN;
    // }
    // flash_timeout.stop();
    // } else {
    // state = State.FLASHING_GREEN;
    // flash_timeout.start();
    // }
    // } else {
    // flash_timeout.reset();
    // if (wantsCube) {
    // state = State.SOLID_PURPLE;
    // } else {
    // state = LEDs.State.SOLID_YELLOW;
    // }
    // }

    // mLEDs.applyStates(state);

    // }

    @Log
    public int requestQueueLength() {
        return queuedRequests.size();
    }

    @Log
    public int sequentialRequestStep() {
        if (activeRequest instanceof SequentialRequest) {
            return ((SequentialRequest) activeRequest).getListLength();
        } else {
            return -1;
        }
    }

    @Log
    public String activeRequest() {
        if (activeRequest == null) {
            return "null";
        } else if (activeRequest instanceof SequentialRequest) {
            return ((SequentialRequest) activeRequest).getActiveRequest();
        } else {
            return activeRequest.toString();
        }
    }
}