package com.team8013.frc2024.subsystems;

import com.team254.lib.util.Util;
import com.team8013.frc2024.Constants;
import com.team8013.frc2024.controlboard.ControlBoard;
import com.team8013.frc2024.controlboard.CustomXboxController.Button;
import com.team8013.frc2024.controlboard.CustomXboxController.Side;
import com.team8013.frc2024.loops.ILooper;
import com.team8013.frc2024.loops.Loop;
import com.team8013.frc2024.subsystems.EndEffectorREV.State;
import com.team8013.lib.Conversions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem {

    private Elevator mElevator = Elevator.getInstance();
    private Wrist mWrist = Wrist.getInstance();
    // private Drive mDrive = Drive.getInstance();
    private Limelight mLimelight = Limelight.getInstance();
    private ControlBoard mControlBoard = ControlBoard.getInstance();
    private Pivot mPivot = Pivot.getInstance();
    private Shooter mShooter = Shooter.getInstance();
    private ClimberHook mClimberHook = ClimberHook.getInstance();
    private EndEffectorREV mEndEffector = EndEffectorREV.getInstance();

    private double pivotManualPosition = mPivot.getPivotAngleDeg() + 4;
    private double elevatorManualPosition = mElevator.getElevatorUnits() + 0.02;
    private double wristManualPosition = mWrist.getWristAngleDeg();
    private double climberHookManualPosition = mClimberHook.getAngleDeg();
    private SuperstructureState mSuperstructureState;

    private boolean manualControlMode;
    private boolean outtake;
    private boolean wantsManualIntake;
    private int climbingTracker = -1;
    private int transfterToShooterTracker = -1;
    private double deClimbTracker = -1;
    private int autoShotTracker = -1;
    private int intakingShooterSourceTracker = -1;
    private int shooterToEndEffectorTracker = -1;
    private int lowPassTracker = -1;
    private boolean climbModeStage2 = false;
    private boolean climbFinished = false;
    private double manualControClimbHeight = Constants.ElevatorConstants.kClimbInitHeight;
    private boolean mWantsToShoot = false;
    private boolean autoShot = false;
    private double gamePieceDelayCounter = 0;
    private Timer shootingTimer = new Timer();


    /* Singleton Instance */
    private static Superstructure mInstance;

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    };

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
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
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    Timer flash_timeout = new Timer();

    public void resetForAuto() {
        autoShotTracker = -1;
    }

    public enum SuperstructureState {
        INTAKING_GROUND,
        INTAKING_SOURCE,
        INTAKING_SHOOTER_SOURCE,
        SCORE_AMP,
        TRANSFER_TO_SHOOTER,
        STOW,
        CLIMB,
        DECLIMB,
        SHOOTER_TO_END_EFFECTOR,
        SHOOTER_TO_AMP,
        LOW_PASS,
        INTAKING_SOURCE_MANUAL
    }

    public void setSuperstuctureIntakingGround() {
        if (mSuperstructureState != SuperstructureState.INTAKING_GROUND
                && (!(mShooter.getBeamBreak() || mEndEffector.hasGamePiece()))) {
            mSuperstructureState = SuperstructureState.INTAKING_GROUND;

            mWrist.setSetpointMotionMagic(280);
            gamePieceDelayCounter = 0;
            // flips the wrist down immediatly, then elevator waits for it to get into
            // position before extending

        }
    }

    public void setSuperstuctureIntakingSource() {
        if (mSuperstructureState != SuperstructureState.INTAKING_SOURCE
                && (!(mShooter.getBeamBreak() || mEndEffector.hasGamePiece()))) {
            mSuperstructureState = SuperstructureState.INTAKING_SOURCE;
        }
    }

    public void setSuperstuctureScoreAmp() {
        if (mSuperstructureState != SuperstructureState.SCORE_AMP
                && mSuperstructureState != SuperstructureState.SHOOTER_TO_AMP && !mShooter.getBeamBreak()) {
            mSuperstructureState = SuperstructureState.SCORE_AMP;
        } else if (mSuperstructureState != SuperstructureState.SHOOTER_TO_AMP && mShooter.getBeamBreak()) {
            mSuperstructureState = SuperstructureState.SHOOTER_TO_AMP;
            shooterToEndEffectorTracker = -1;
        }
    }

    public void setSuperstuctureLowPass() {
        if (mSuperstructureState == SuperstructureState.STOW && mShooter.getBeamBreak()) {
            mSuperstructureState = SuperstructureState.LOW_PASS;
            lowPassTracker = -1;
            shootingTimer.reset();
        }
    }

    public void setSuperstuctureTransferToShooter() {
        if (mSuperstructureState != SuperstructureState.TRANSFER_TO_SHOOTER) {
            mSuperstructureState = SuperstructureState.TRANSFER_TO_SHOOTER;

            if (!mShooter.getBeamBreak()) {
                transfterToShooterTracker = -1;
            } else {
                transfterToShooterTracker = 1;
                mWrist.setSetpointMotionMagic(Constants.WristConstants.kloadShooterAngle);
            }
            mWantsToShoot = false;
            shootingTimer.reset();
        }
    }

    public void setSuperstuctureShoot(boolean shoot) {
        mWantsToShoot = shoot;
    }

    public void setSuperstuctureStow() {
        if (mSuperstructureState != SuperstructureState.STOW) {
            mSuperstructureState = SuperstructureState.STOW;
            mPivot.setSetpointMotionMagic(14);
        }
    }

    public void setSuperstuctureSourceLoadShooter() {
        if (mSuperstructureState != SuperstructureState.INTAKING_SHOOTER_SOURCE && !mShooter.getBeamBreak()) {
            mSuperstructureState = SuperstructureState.INTAKING_SHOOTER_SOURCE;
            intakingShooterSourceTracker = -1;
            if (mEndEffector.hasGamePiece()) {
                intakingShooterSourceTracker = 2;
            }
        }

    }

    public void setSuperstuctureShooterToEndEffector() {
        if (mSuperstructureState != SuperstructureState.SHOOTER_TO_END_EFFECTOR && mShooter.getBeamBreak()) {
            mSuperstructureState = SuperstructureState.SHOOTER_TO_END_EFFECTOR;
            shooterToEndEffectorTracker = -1;
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
            // reset everything
            climbingTracker = -1;
            climbModeStage2 = false;
            climbFinished = false;
            manualControClimbHeight = Constants.ElevatorConstants.kClimbInitHeight;
        }
    }

    public void setManualSourceIntake() {
        if (mSuperstructureState != SuperstructureState.INTAKING_SOURCE_MANUAL) {
            mSuperstructureState = SuperstructureState.INTAKING_SOURCE_MANUAL;
        }
    }

    public void setClimbModeStage2() {
        if (mSuperstructureState == SuperstructureState.CLIMB) {
            climbModeStage2 = true;
        }
    }

    public boolean climbFinished() {
        return climbFinished;
    }

    public void setSuperstuctureDeclimb() {
        if (mSuperstructureState != SuperstructureState.DECLIMB) {
            mSuperstructureState = SuperstructureState.DECLIMB;
        }
        deClimbTracker = -1;
    }

    public boolean isDeclimbing() {
        return mSuperstructureState == SuperstructureState.DECLIMB;
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

    public void disableAutoShot() {
        autoShot = false;
    }

    @Override
    public void writePeriodicOutputs() {
        if (manualControlMode) {
            // do manual control things -- good for early season testing and finding
            // position setpoints
        } else {
            if (mSuperstructureState == SuperstructureState.STOW) {
                SmartDashboard.putString("SUPERSTRUCTURE STATE: ", "STOW");

                mPivot.setSetpointMotionMagic(Constants.PivotConstants.kStowAngle);
                if (mShooter.getBeamBreak()) {
                    mElevator.setSetpointMotionMagic(
                            Constants.ElevatorConstants.kStowHeight + Conversions.inchesToMeters(0.25));
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kloadShooterAngle + 1.5);
                    mShooter.setOpenLoopDemand(-0.03);
                } else {
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kStowAngle);
                    mShooter.setOpenLoopDemand(0);
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kStowHeight);
                }

            } else if (mSuperstructureState == SuperstructureState.TRANSFER_TO_SHOOTER) {
                SmartDashboard.putString("SUPERSTRUCTURE STATE: ", "TRANSFER_TO_SHOOTER");

                /*
                 * Steps:
                 * 0: set elevaotr, pivot, and wrist to load shooter initial angle
                 * 1: once elevator is fully out enough to clear the ring, bring it back in to
                 * load the ring in, once it has gone far enough in, eject it
                 * 2: once the shooter detects a note loaded, go immediatly to shooting
                 */

                if (transfterToShooterTracker == -1 && (mShooter.getBeamBreak() || mEndEffector.hasGamePiece())) {
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kloadShooterAngle + 1);
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kloadShooterInitialHeight);
                    mPivot.setSetpointMotionMagic(Util.limit(Constants.PivotConstants.kShootLoadAngle - 5,
                            Constants.PivotConstants.kShootLoadAngle + 8, mLimelight.getPivotShootingAngle()));
                    if (!mShooter.getBeamBreak()) {
                        mShooter.setOpenLoopDemand(Constants.ShooterConstants.kLoadShooterDemand);
                    } else {
                        transfterToShooterTracker = 1;
                        mEndEffector.setOpenLoopDemand(-0.9, -0.9);
                        // ejects note into the shooter if it's stuck in between
                    }
                    transfterToShooterTracker = 0;
                } else if (transfterToShooterTracker == -1) {
                    mShooter.setOpenLoopDemand(-0.5);
                }

                if ((transfterToShooterTracker == 0)
                        && (mElevator.getElevatorUnits() > Constants.ElevatorConstants.kloadShooterInitialHeight
                                - Constants.ElevatorConstants.kPositionError)) {
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kloadShooterFinalHeight);
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kloadShooterAngle);
                    // mPivot.setSetpointMotionMagic(Constants.PivotConstants.kShootAgainstSubwooferAngle);

                    transfterToShooterTracker = 1;
                }

                // mShooterLoaded = mShooter.getBeamBreak();

                if ((transfterToShooterTracker == 1)
                        && (mElevator.getElevatorUnits() < Constants.ElevatorConstants.kloadShooterFinalHeight
                                + Constants.ElevatorConstants.kPositionError)
                        && (!mShooter.getBeamBreak())) {
                    mEndEffector.setOpenLoopDemand(-0.9, -0.9);
                    // ejects note into the shooter if it's stuck in between
                }

                if (mShooter.getBeamBreak() && transfterToShooterTracker == 1) {
                    mShooter.setOpenLoopDemand(-0.03);
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kloadShooterAngle - 0.5);
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kShootHeight);
                    // mEndEffector.setOpenLoopDemand(0.95); //HERE DO RPM

                    transfterToShooterTracker = 2;
                }

                if (transfterToShooterTracker == 2
                        && mElevator.getElevatorUnits() > Constants.ElevatorConstants.kStowHeight
                                + Conversions.inchesToMeters(2)) {
                    // determine end effector rpm here
                    mEndEffector.setEndEffectorClosedLoop(mLimelight.getEndEffectorShootingVelocity());
                }

                if (transfterToShooterTracker == 2) {
                    /* Manual control in the limelight class */
                    mPivot.setSetpointMotionMagic(mLimelight.getPivotShootingAngle());
                }

                if ((transfterToShooterTracker == 2) && mWantsToShoot
                        && (mElevator.getElevatorUnits() > Constants.ElevatorConstants.kShootHeight
                                - Constants.ElevatorConstants.kPositionError)
                        && (Util.epsilonEquals(mPivot.getPivotAngleDeg(), mLimelight.getPivotShootingAngle(),
                                Constants.PivotConstants.kPositionError))
                        &&
                        ((mEndEffector.getVelocityMaster() - mLimelight.getEndEffectorShootingVelocity()) > -600)) {
                    mShooter.setOpenLoopDemand(Constants.ShooterConstants.kSlingshotDemand);
                    transfterToShooterTracker = 3;
                }

                if ((transfterToShooterTracker == 3) && (!mShooter.getBeamBreak())) {
                    shootingTimer.stop();
                    shootingTimer.reset();
                    shootingTimer.start();
                    transfterToShooterTracker = 4;
                }

                if ((transfterToShooterTracker == 4) && (shootingTimer.get() > 0.2)) {
                    shootingTimer.stop();
                    shootingTimer.reset();
                    // done shooting
                    mWantsToShoot = false;
                    // mEndEffector.setState(State.IDLE);
                    mSuperstructureState = SuperstructureState.STOW;
                }

            } else if (mSuperstructureState == SuperstructureState.SCORE_AMP) {
                SmartDashboard.putString("SUPERSTRUCTURE STATE: ", "SCORE AMP");

                mWrist.setSetpointMotionMagic(Constants.WristConstants.kAmpScoreAngle);
                mPivot.setSetpointMotionMagic(Constants.PivotConstants.kAmpScoreAngle);

                if (mPivot.getPivotAngleDeg() > 25) {
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kAmpScoreHeight);
                }

                if (!mEndEffector.hasGamePiece()) {
                    // automatically stow after game piece ejected
                    mSuperstructureState = SuperstructureState.STOW;
                }

            } else if (mSuperstructureState == SuperstructureState.INTAKING_GROUND) {
                SmartDashboard.putString("SUPERSTRUCTURE STATE: ", "INTAKING GROUND");

                if (mWrist.getWristAngleDeg() > 13) { // greater angle, furthur down 225
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kFloorIntakeHeight);
                    // need to follow an array so that it doesn't extend beyond the extension limits
                    mWrist.setSetpointMotionMagic(getPositionsGroundIntakeOut(mElevator.getElevatorUnits())[0]);
                    mPivot.setSetpointMotionMagic(getPositionsGroundIntakeOut(mElevator.getElevatorUnits())[1]);
                }

                if (!mEndEffector.hasGamePiece() && mWrist.getWristAngleDeg() > 350) {
                    mEndEffector.setState(State.INTAKING);
                } else if (mEndEffector.hasGamePiece()) {
                    gamePieceDelayCounter = gamePieceDelayCounter + 1;

                }
                if (gamePieceDelayCounter > 1) {
                    mEndEffector.setState(State.IDLE);
                    // Once game piece aquired, then stow
                    mSuperstructureState = SuperstructureState.STOW;
                }

            } else if (mSuperstructureState == SuperstructureState.INTAKING_SOURCE) {
                SmartDashboard.putString("SUPERSTRUCTURE STATE: ", "INTAKING SOURCE");
                mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kSourceIntakeHeight);
                mPivot.setSetpointMotionMagic(Constants.PivotConstants.kSourceIntakeAngle);

                if (mPivot.getPivotAngleDeg() > Constants.PivotConstants.kSourceIntakeAngle - 55) {
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kSourceIntakeAngle);
                }

                if (!mEndEffector.hasGamePiece() && mWrist.getWristAngleDeg() > 260) {
                    mEndEffector.setOpenLoopDemand(Constants.EndEffectorConstants.kSourceIntakeDemand); /// .48 seemed
                                                                                                        /// to work
                                                                                                        /// //.41 last
                                                                                                        /// comp
                    // this value changes based on the rolling resistance of the end effector tubes
                } else if (mEndEffector.hasGamePiece()) {
                    mEndEffector.setOpenLoopDemand(0.0);
                    // Once game piece aquired, then stow
                    mSuperstructureState = SuperstructureState.STOW;
                }

            } else if (mSuperstructureState == SuperstructureState.CLIMB) {
                SmartDashboard.putString("SUPERSTRUCTURE STATE: ", "CLIMB");

                /* CLIMB STATE YAY */

                /*
                 * Climbing Tracker
                 * 0: climb Setup -- ready to hook on
                 * 1: halfway pulled onto chain, ready for pivot
                 * 2: hooked onto chain, ready for pivot to go up and elevator to extend
                 * 3: unhooked elevator from chain, ready for user input to raise up and score
                 * in trap
                 * 4: elevator and pivot extended out, ready for quick pivot
                 * 5:
                 * 
                 * 
                 * New Version -- 2024 Houston and NE DCMP
                 * 
                 * -1: initialize climb, extend pivot up under chain, bring hooks up to 50
                 * degrees
                 * 0: y has been pressed, elevator extends to hook on chain (height manually
                 * adjustable at this point)
                 * 2: both bumpers pressed, elevator and pivot come down to hook the chain onto
                 * the climber hooks
                 * 3: triggers pressed or held, elevator and pivot extend back up to reach into
                 * the trap and score
                 */

                if (climbingTracker < 1 && mControlBoard.operator.getController().getPOV() == 270) {// if want stow go
                                                                                                    // back to stow
                    mSuperstructureState = SuperstructureState.STOW;
                }

                // Stage 1: set up climb
                if (climbingTracker == -1) { // bot is in position, raise pivot up under chain
                    mElevator.setSetpointMotionMagic(Conversions.inchesToMeters(0.5));
                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kClimbInitAngle1);
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kClimbAngle1);

                }

                if (climbingTracker == -1 && mPivot.getPivotAngleDeg() > 55
                        && mControlBoard.operator.getButton(Button.X)) {
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kClimbInitHeight);
                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kClimbInitAngle2);
                    // go to an init position to speed up trap climb
                    mClimberHook.setSetpointMotionMagic(50);
                    climbingTracker = 0;
                }

                // Stage 2: once climb set up, wait for user to press button to pull down to
                // chain CURL
                if ((climbModeStage2) && (climbingTracker == 0) && !(Util.epsilonEquals(mElevator.getElevatorUnits(),
                        Constants.ElevatorConstants.kClimbInitHeight, Constants.ElevatorConstants.kPositionError))) {
                    // if elevator is not in place, make it
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kClimbInitHeight);
                } else if ((climbModeStage2) && (climbingTracker == 0)
                        && (Util.epsilonEquals(mElevator.getElevatorUnits(),
                                Constants.ElevatorConstants.kClimbInitHeight,
                                Constants.ElevatorConstants.kPositionError))
                        && (mPivot.getPivotAngleDeg() > Constants.PivotConstants.kClimbInitAngle2
                                - Constants.PivotConstants.kPositionError)) {

                    // set motor configs, custom velocities while climbing
                    mElevator.setMotorConfig(Constants.ElevatorConstants.elevatorCurlMotorConfig());
                    mPivot.setMotorConfig(Constants.PivotConstants.pivotCurlMotorConfig());
                    // mWrist.setMotorConfig(Constants.WristConstants.wristMotorClimbConfig());

                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kPullOntoChainHeight);
                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kPullOntoChainAngle2);
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kClimbFirstPressAngle);
                    climbingTracker = 2;
                } else if (climbingTracker == 0) { // manual control height
                    if (mControlBoard.operator.getController().getRightY() > 0.2) {
                        manualControClimbHeight -= 0.0025;
                    } else if (mControlBoard.operator.getController().getRightY() < -0.2) {
                        manualControClimbHeight += 0.0025;
                    }
                    manualControClimbHeight = Util.limit(manualControClimbHeight,
                            Constants.ElevatorConstants.kMaxClimbInitHeight);
                    mElevator.setSetpointMotionMagic(manualControClimbHeight);
                }

                if (climbingTracker == 1) { // does nothing, used to have previous steps
                    climbingTracker = 2;
                }

                if (climbingTracker == 2 && mPivot.getPivotAngleDeg() < Constants.PivotConstants.kPullOntoChainAngle2
                        + 1) {
                    mClimberHook.setSetpointMotionMagic(95); // 92 WORKS
                    climbingTracker = 3;
                }

                if (climbingTracker == 3 && (mControlBoard.operator.getTrigger(Side.LEFT)
                        && mControlBoard.operator.getTrigger(Side.RIGHT)) && mClimberHook.getAngleDeg() > 86) {
                    mElevator.setMotorConfig(Constants.ElevatorConstants.elevatorSlowMotorConfig());
                    mPivot.setMotorConfig(Constants.PivotConstants.pivotSlowMotorConfig());
                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kExtendToScoreTrapAngle2);
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kExtendOffChain3);
                    // mClimberHook.setSetpointMotionMagic(80); //bring value up to help press the
                    // robot against the wall
                    climbingTracker = 4;
                }

                // Stage 3: wait for user to press button to extend up to trap
                if ((climbingTracker == 4)
                        && (mPivot.getPivotAngleDeg() > Constants.PivotConstants.kExtendOffChainAngle1
                                - 4)) {
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kExtendOffChain3);
                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kExtendOffChainAngle2);

                    climbingTracker = 5;
                }

                if ((climbingTracker == 5)
                        && (mElevator.getElevatorUnits() > Constants.ElevatorConstants.kExtendOffChain2
                                - Constants.ElevatorConstants.kPositionError)) {

                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kClimbSecondPressAngle);
                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kExtendToScoreTrapAngle1); // fast
                }

                if (climbingTracker == 5
                        && mPivot.getPivotAngleDeg() > Constants.PivotConstants.kExtendOffChainAngle2 - 2) {
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kExtendOffChain3);
                    climbingTracker = 6;
                }

                if ((climbingTracker == 6)
                        &&
                        (mElevator.getElevatorUnits() > Constants.ElevatorConstants.kExtendOffChain3
                                - Constants.ElevatorConstants.kPositionError)) {

                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kExtendToScoreTrapAngle2);
                    // mWrist.setSetpointMotionMagic(Constants.WristConstants.kClimbScoreInTrapAngle);

                    // on the trap wall pressed against it, maybe shake the pivot to wedge the end
                    // effector in
                }
                if ((climbingTracker == 6)
                        && mPivot.getPivotAngleDeg() > Constants.PivotConstants.kExtendToScoreTrapAngle2 - 3) {
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kExtendToScoreTrapHeight);
                    climbingTracker = 7;
                }
                if ((climbingTracker == 7) &&
                        (mElevator.getElevatorUnits() > Constants.ElevatorConstants.kExtendToScoreTrapHeight
                                - Conversions.inchesToMeters(7.7))) {
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kClimbScoreInTrapAngle);
                    climbingTracker = 8;
                }

                if ((climbingTracker == 8) && (Util.epsilonEquals(mWrist.getWristAngleDeg(),
                        Constants.WristConstants.kClimbScoreInTrapAngle, 4)
                        || (mElevator.getElevatorUnits() > Constants.ElevatorConstants.kExtendToScoreTrapHeight
                                - Constants.ElevatorConstants.kPositionError))) {
                    // lets down below know when outtaking is allowed so we can hold down eject
                    // while climbing to get the quickest trap
                    climbFinished = true;
                }

            } else if (mSuperstructureState == SuperstructureState.DECLIMB) {
                SmartDashboard.putString("SUPERSTRUCTURE STATE: ", "DECLIMB");

                if (deClimbTracker == -1) {
                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kDeclimbAngle3); // 1
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kDeclimbHeight3); // 1
                    deClimbTracker = 1;
                }

                // old version of the climb was slower because it limited movement of motors
                // until they reached setpoints,
                // just tuning the motor velocities is smoother - quicker, and more simple

                if ((deClimbTracker == 1) && (mPivot.getPivotAngleDeg() < Constants.PivotConstants.kDeclimbAngle2
                        + Constants.PivotConstants.kPositionError)) {
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kShootAngle + 1.5);
                    mClimberHook.setSetpointMotionMagic(Constants.ClimberHookConstants.kDeclimb1Angle);
                }

                if ((deClimbTracker == 1) && (mElevator.getElevatorUnits() < Constants.ElevatorConstants.kDeclimbHeight2
                        + Constants.ElevatorConstants.kPositionError)) {
                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kDeclimbAngle3);
                }

                if ((deClimbTracker == 1)
                        && (mPivot.getPivotAngleDeg() < Constants.PivotConstants.kDeclimbAngle3 + 1.5)) {
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kDeclimbHeight3);
                    deClimbTracker = 2;
                }

                if ((mControlBoard.operator.getButton(Button.X)) && (deClimbTracker == 2)) {
                    mClimberHook.setSetpointMotionMagic(Constants.ClimberHookConstants.kUnhookAngle);
                    deClimbTracker = 3;
                }

                if (deClimbTracker == 3
                        && mClimberHook.getAngleDeg() < Constants.ClimberHookConstants.kUnhookAngle + 2) {
                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kDeclimbAngle4);
                    deClimbTracker = 4;
                }

                if (deClimbTracker == 4 && mPivot.getPivotAngleDeg() > 25) {
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kDeclimbHeight4);
                    manualControClimbHeight = Constants.ElevatorConstants.kDeclimbHeight4;
                    deClimbTracker = 5;
                }

                if (deClimbTracker == 5) {
                    if (mControlBoard.operator.getController().getRightY() > 0.2) {
                        manualControClimbHeight -= 0.0025;
                    } else if (mControlBoard.operator.getController().getRightY() < -0.2) {
                        manualControClimbHeight += 0.0025;
                    }
                    manualControClimbHeight = Util.limit(manualControClimbHeight,
                            Constants.ElevatorConstants.kMaxClimbInitHeight);
                    mElevator.setSetpointMotionMagic(manualControClimbHeight);
                }

                if (deClimbTracker == 5 && (mControlBoard.operator.getController().getPOV() == 270
                        || mControlBoard.operator.getButton(Button.Y))) {
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kStowHeight);
                    deClimbTracker = 6;
                }

                if ((deClimbTracker == 6) && (mElevator.getElevatorUnits() < Constants.ElevatorConstants.kStowHeight
                        + Constants.ElevatorConstants.kPositionError)) {
                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kStowAngle);
                    deClimbTracker = 7;
                }

                if ((deClimbTracker == 7) && (mPivot.getPivotAngleDeg() < Constants.PivotConstants.kStowAngle
                        + Constants.PivotConstants.kPositionError)) {
                    mPivot.setMotorConfig(Constants.PivotConstants.pivotFastMotorConfig());
                    mElevator.setMotorConfig(Constants.ElevatorConstants.elevatorFastMotorConfig());
                    mSuperstructureState = SuperstructureState.STOW;
                }

            } else if (mSuperstructureState == SuperstructureState.INTAKING_SHOOTER_SOURCE) {
                SmartDashboard.putString("SUPERSTRUCTURE STATE: ", "INTAKING SHOOTER SOURCE");

                if (intakingShooterSourceTracker == -1) {
                    mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kSourceIntakeHeight);
                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kSourceIntakeAngle);
                    transfterToShooterTracker = -1;
                }

                if (mPivot.getPivotAngleDeg() > Constants.PivotConstants.kSourceIntakeAngle - 55
                        && intakingShooterSourceTracker == -1) {
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kSourceIntakeAngle);
                    intakingShooterSourceTracker = 1;
                }

                if (!mEndEffector.hasGamePiece() && mWrist.getWristAngleDeg() > 260
                        && intakingShooterSourceTracker == 1) {
                    mEndEffector.setOpenLoopDemand(Constants.EndEffectorConstants.kSourceIntakeDemand);
                    intakingShooterSourceTracker = 2;
                } else if (mEndEffector.hasGamePiece() && intakingShooterSourceTracker == 2) {
                    mEndEffector.setOpenLoopDemand(0.0);
                    intakingShooterSourceTracker = 3;
                    transfterToShooterTracker = -1;
                    // Once game piece aquired, then stow
                }
                if (transfterToShooterTracker == -1 && intakingShooterSourceTracker == 3) {
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kloadShooterAngle);
                    mElevator.setSetpointMotionMagic(
                            Constants.ElevatorConstants.kloadShooterInitialHeight + Conversions.inchesToMeters(0.5));
                    mPivot.setSetpointMotionMagic(60);
                    mShooter.setOpenLoopDemand(Constants.ShooterConstants.kLoadShooterDemand);
                    transfterToShooterTracker = 0;
                }

                if ((transfterToShooterTracker == 0)
                        && (mElevator.getElevatorUnits() > (Constants.ElevatorConstants.kloadShooterInitialHeight
                                + Conversions.inchesToMeters(0.5)
                                - Constants.ElevatorConstants.kPositionError))) {
                    mElevator.setSetpointMotionMagic(
                            Constants.ElevatorConstants.kStowHeight + Conversions.inchesToMeters(1));
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kloadShooterAngle);

                    transfterToShooterTracker = 1;
                }

                if ((transfterToShooterTracker == 1)
                        && (mElevator.getElevatorUnits() < (Constants.ElevatorConstants.kloadShooterInitialHeight
                                - Conversions.inchesToMeters(0.5)))) {
                    mPivot.setSetpointMotionMagic(Constants.PivotConstants.kStowAngle);
                }

                if ((transfterToShooterTracker == 1)
                        && (mElevator.getElevatorUnits() < (Constants.ElevatorConstants.kloadShooterFinalHeight
                                - Conversions.inchesToMeters(1)
                                + Constants.ElevatorConstants.kPositionError))
                        && (!mShooter.getBeamBreak())) {
                    mEndEffector.setOpenLoopDemand(-0.15, -0.17);
                    // change this too

                }

                if (mShooter.getBeamBreak() && transfterToShooterTracker == 1) {
                    mShooter.setOpenLoopDemand(-0.005);
                    mEndEffector.setState(State.IDLE);
                    transfterToShooterTracker = 2;
                }
                if (transfterToShooterTracker == 2
                        && Util.epsilonEquals(Constants.PivotConstants.kStowAngle, mPivot.getPivotAngleDeg(), 4)) {
                    // if within 4 degrees of stowing, just put in stow
                    mSuperstructureState = SuperstructureState.STOW;
                }

            } else if (mSuperstructureState == SuperstructureState.SHOOTER_TO_END_EFFECTOR) {

                if (shooterToEndEffectorTracker == -1) {
                    mElevator.setSetpointMotionMagic(Conversions.inchesToMeters(6));
                    mPivot.setSetpointMotionMagic(15);
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kloadShooterAngle);
                    shooterToEndEffectorTracker = 0;
                }
                if ((mElevator.getElevatorUnits() > Conversions.inchesToMeters(6)
                        - Constants.ElevatorConstants.kPositionError) &&
                        (mWrist.getWristAngleDeg() < Constants.WristConstants.kloadShooterAngle + 2)
                        && shooterToEndEffectorTracker == 0) {
                    mShooter.setOpenLoopDemand(0.85);
                    mEndEffector.setOpenLoopDemand(0.4);
                    shooterToEndEffectorTracker = 1;
                }
                if (shooterToEndEffectorTracker == 1 && mEndEffector.hasGamePiece()) {
                    mShooter.setOpenLoopDemand(0);
                    mEndEffector.setOpenLoopDemand(0);
                    mSuperstructureState = SuperstructureState.STOW;
                }

            } else if (mSuperstructureState == SuperstructureState.SHOOTER_TO_AMP) {
                if (shooterToEndEffectorTracker == -1) {
                    mElevator.setSetpointMotionMagic(Conversions.inchesToMeters(6));
                    mPivot.setSetpointMotionMagic(15);
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kloadShooterAngle);
                    shooterToEndEffectorTracker = 0;
                }
                if ((mElevator.getElevatorUnits() > Conversions.inchesToMeters(6)
                        - Constants.ElevatorConstants.kPositionError) &&
                        (mWrist.getWristAngleDeg() < Constants.WristConstants.kloadShooterAngle + 2)
                        && shooterToEndEffectorTracker == 0) {
                    mShooter.setOpenLoopDemand(0.9);
                    mEndEffector.setOpenLoopDemand(0.4);
                    shooterToEndEffectorTracker = 1;
                }
                if (shooterToEndEffectorTracker == 1 && mEndEffector.hasGamePiece()) {
                    mShooter.setOpenLoopDemand(0);
                    mEndEffector.setOpenLoopDemand(0);
                    mSuperstructureState = SuperstructureState.SCORE_AMP;
                }
            } else if (mSuperstructureState == SuperstructureState.LOW_PASS) {
                if (lowPassTracker == -1) {
                    mElevator.setSetpointMotionMagic(
                            Constants.ElevatorConstants.kStowHeight + Conversions.inchesToMeters(1.5));
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kShootAngle);
                    mPivot.setSetpointMotionMagic(10);
                    lowPassTracker = 0;
                }
                if (mElevator.getElevatorUnits() > Constants.ElevatorConstants.kStowHeight
                        + Conversions.inchesToMeters(1)) {
                    mEndEffector.setOpenLoopDemand(0.95);
                }
                if (mEndEffector.getVelocityMaster() > 4000 && lowPassTracker == 0) {
                    mShooter.setOpenLoopDemand(Constants.ShooterConstants.kSlingshotDemand);
                    shootingTimer.stop();
                    shootingTimer.reset();
                    shootingTimer.start();
                    lowPassTracker = 1;
                } else if (lowPassTracker == 0) {
                    mShooter.setOpenLoopDemand(-0.01);
                }
                if (!mShooter.getBeamBreak() && lowPassTracker == 1 && shootingTimer.get() > 0.2) {
                    mShooter.setOpenLoopDemand(0);
                    mSuperstructureState = SuperstructureState.STOW;
                }
            } else if (mSuperstructureState == SuperstructureState.INTAKING_SOURCE_MANUAL) {
                // added after the match with 5940 where our end effector got smashed and the
                // beam break triggered so we couldn't intake
                SmartDashboard.putString("SUPERSTRUCTURE STATE: ", "INTAKING SOURCE MANUAL");
                mElevator.setSetpointMotionMagic(Constants.ElevatorConstants.kSourceIntakeHeight);
                mPivot.setSetpointMotionMagic(Constants.PivotConstants.kSourceIntakeAngle);

                if (mPivot.getPivotAngleDeg() > Constants.PivotConstants.kSourceIntakeAngle - 55) {
                    mWrist.setSetpointMotionMagic(Constants.WristConstants.kSourceIntakeAngle);
                }

            }

            if (mSuperstructureState == SuperstructureState.STOW && mShooter.getBeamBreak()
                    && (mControlBoard.operator.getController().getPOV() == 0)) {
                setSuperstuctureLowPass();
            } else if (((mSuperstructureState == SuperstructureState.CLIMB && climbFinished) // not allowed to eject
                                                                                             // until
                    // climb is done
                    || mSuperstructureState == SuperstructureState.SCORE_AMP
                    || mSuperstructureState == SuperstructureState.STOW)
                    && (mControlBoard.operator.getController().getPOV() == 0)) {
                mEndEffector.setOpenLoopDemand(-0.4);
            } else if (mSuperstructureState == SuperstructureState.CLIMB
                    || mSuperstructureState == SuperstructureState.SCORE_AMP
                    || mSuperstructureState == SuperstructureState.STOW
                            && outsideError(mControlBoard.operator.getController().getLeftX(), 0.2)) {
                mEndEffector.setOpenLoopDemand(mControlBoard.operator.getController().getLeftX() * 0.075);
            } else if (mSuperstructureState == SuperstructureState.INTAKING_SOURCE_MANUAL
                    && outsideError(mControlBoard.operator.getController().getLeftX(), 0.2)) {
                mEndEffector.setOpenLoopDemand(mControlBoard.operator.getController().getLeftX() * 0.075);
            } else if (// mSuperstructureState == SuperstructureState.CLIMB ||
            mSuperstructureState == SuperstructureState.SCORE_AMP
                    || mSuperstructureState == SuperstructureState.STOW
                    || mSuperstructureState == SuperstructureState.INTAKING_SOURCE_MANUAL) {
                mEndEffector.setOpenLoopDemand(0);
            }

            if (mSuperstructureState != SuperstructureState.TRANSFER_TO_SHOOTER
                    && mSuperstructureState != SuperstructureState.INTAKING_SHOOTER_SOURCE
                    && mSuperstructureState != SuperstructureState.SHOOTER_TO_END_EFFECTOR
                    && mSuperstructureState != SuperstructureState.SHOOTER_TO_AMP
                    && mSuperstructureState != SuperstructureState.STOW
                    && mSuperstructureState != SuperstructureState.LOW_PASS) {
                mShooter.setOpenLoopDemand(0);
            }
        }

        if (autoShot && autoShotTracker == -1)

        {
            if (mSuperstructureState != SuperstructureState.TRANSFER_TO_SHOOTER
                    && (mEndEffector.hasGamePiece() || mShooter.getBeamBreak())) {
                setSuperstuctureTransferToShooter();
                // autoShotTracker = 0;

            } else if (mShooter.getBeamBreak()) {
                autoShotTracker = 0;
            }
        }

        if ((Util.epsilonEquals(mEndEffector.getVelocityMaster(), mLimelight.getEndEffectorShootingVelocity(), 600))
                && autoShotTracker == 0 && (mPivot.getPivotAngleDeg() > (mLimelight.getPivotShootingAngle() - 1.5))
                && mShooter.getBeamBreak() && autoShot) {
            setSuperstuctureShoot(true);
            autoShotTracker = 1;
            autoShot = false;
        }

    }

    // this already exists in util.kepsilonEquals
    public boolean outsideError(double a, double error) {
        return ((Math.abs(a) - Math.abs(error)) > 0);
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
        for (int i = 0; i < Constants.ElevatorConstants.groundIntakeWristPositionsOut.length; i++) {
            if (Constants.ElevatorConstants.groundIntakeWristPositionsOut[i][0] < elevatorPosition) {
                index = i;
            }
        }
        return new double[] { Constants.ElevatorConstants.groundIntakeWristPositionsOut[index][1],
                Constants.ElevatorConstants.groundIntakeWristPositionsOut[index][2] };
    }

    // private double[] getPositionsGroundIntakeIn(double elevatorPosition) {

    // int index = 0;
    // for (int i = Constants.ElevatorConstants.groundIntakeWristPositionsIn.length
    // - 1; i >= 0; i--) {
    // if (Constants.ElevatorConstants.groundIntakeWristPositionsIn[i][0] >
    // elevatorPosition) {
    // index = i;
    // }
    // }
    // return new double[] {
    // Constants.ElevatorConstants.groundIntakeWristPositionsIn[index][1] - 10,
    // Constants.ElevatorConstants.groundIntakeWristPositionsIn[index][2] + 1 };
    // }

    public void autoShot() {
        autoShot = true;
        autoShotTracker = -1;
    }

    public boolean hasGamePiece() {
        return (mShooter.getBeamBreak() || mEndEffector.hasGamePiece());
    }

    // Manual controls used in early season testing:

    public void controlPivotManually(double demand) {
        // double position = mPivot.getPivotAngleDeg();
        if (demand > 0.3) {
            pivotManualPosition += 0.3;
        } else if (demand < -0.3) { // &&(pivotManualPosition>Constants.PivotConstants.kMinAngle)
            pivotManualPosition = pivotManualPosition - 0.3;
        }

        mPivot.setSetpointMotionMagic(pivotManualPosition);
    }

    public void controlElevatorManually(double demand) {
        if ((demand > 0.2)) {
            elevatorManualPosition += 0.0015;
        } else if ((demand < -0.2)) { // &&(pivotManualPosition>Constants.PivotConstants.kMinAngle)
            elevatorManualPosition += -0.0015;
        }

        mElevator.setSetpointMotionMagic(elevatorManualPosition);
    }

    public void controlWristManually(double demand) {
        if (demand > 0) {// &&(position<Constants.WristConstants.kMaxPosition)){
            wristManualPosition += 0.4;
        } else if (demand < 0) {// &&(position<Constants.WristConstants.kMinPosition)){
            wristManualPosition = wristManualPosition - 0.4;
        }
        // SmartDashboard.putNumber("wristManualPosition", wristManualPosition);
        mWrist.setSetpointMotionMagic(wristManualPosition);
    }

    public void controlClimberHookManually(double demand) {
        if (demand > 0) {// &&(position<Constants.WristConstants.kMaxPosition)){
            climberHookManualPosition += 0.4;
        } else if (demand < 0) {// &&(position<Constants.WristConstants.kMinPosition)){
            climberHookManualPosition = climberHookManualPosition - 0.4;
        }
        // SmartDashboard.putNumber("wristManualPosition", wristManualPosition);
        mClimberHook.setSetpointMotionMagic(climberHookManualPosition);
    }

}