// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team8013.frc2024;

import java.nio.Buffer;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team254.lib.util.Util;
import com.team8013.frc2024.auto.AutoModeBase;
import com.team8013.frc2024.auto.AutoModeExecutor;
import com.team8013.frc2024.auto.AutoModeSelector;
import com.team8013.frc2024.controlboard.ControlBoard;
import com.team8013.frc2024.controlboard.CustomXboxController.Button;
import com.team8013.frc2024.controlboard.CustomXboxController.Side;
import com.team8013.frc2024.loops.CrashTracker;
import com.team8013.frc2024.loops.Looper;
import com.team8013.frc2024.shuffleboard.ShuffleBoardInteractions;
import com.team8013.frc2024.subsystems.ClimberHook;
import com.team8013.frc2024.subsystems.Drive;
import com.team8013.frc2024.subsystems.Elevator;
import com.team8013.frc2024.subsystems.EndEffectorREV;
//import com.team8013.frc2024.subsystems.EndEffectorREV;
import com.team8013.frc2024.subsystems.Limelight;
import com.team8013.frc2024.subsystems.Pivot;
import com.team8013.frc2024.subsystems.Shooter;
import com.team8013.frc2024.subsystems.Superstructure;
import com.team8013.frc2024.subsystems.Wrist;
//import com.team8013.frc2024.subsystems.EndEffectorREV.State;
import com.team8013.lib.swerve.ChassisSpeeds;

public class Robot extends TimedRobot {

	// util instances
	private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
	private final ControlBoard mControlBoard = ControlBoard.getInstance();
	private final ShuffleBoardInteractions mShuffleboard = ShuffleBoardInteractions.getInstance();
	// private final LoggingSystem mLogger = LoggingSystem.getInstance();

	// subsystem instances
	private final Superstructure mSuperstructure = Superstructure.getInstance();
	private final Drive mDrive = Drive.getInstance();
	private final Limelight mLimelight = Limelight.getInstance();
	private final Pivot mPivot = Pivot.getInstance();
	private final Elevator mElevator = Elevator.getInstance();
	private final Wrist mWrist = Wrist.getInstance();
	private final EndEffectorREV mEndEffector = EndEffectorREV.getInstance();
	private final Shooter mShooter = Shooter.getInstance();
	private final ClimberHook mClimberHook = ClimberHook.getInstance();

	// instantiate enabled and disabled loopers
	private final Looper mEnabledLooper = new Looper();
	private final Looper mDisabledLooper = new Looper();
	// private final Looper mLoggingLooper = new Looper(0.002);

	// auto instances
	private AutoModeExecutor mAutoModeExecutor;
	public final static AutoModeSelector mAutoModeSelector = new AutoModeSelector();

	public static boolean is_red_alliance = false;
	public static boolean flip_trajectories = false;
	public static boolean wantChase = false;
	public static boolean doneChasing = true;
	public static boolean shootFromPodiumBoolean = false;
	private Timer mManualSourceIntakeTimer = new Timer();
	private boolean mManualSourceBoolean = false;
	// private boolean autoAllignBoolean = false;

	// private final int kDpadUp = 0;
	private final int kDpadRight = 90;
	private final int kDpadDown = 180;
	private final int kDpadLeft = 270;

	public Robot() {
		CrashTracker.logRobotConstruction();
	}

	@Override
	public void robotInit() {
		try {
			CrashTracker.logRobotInit();

			LiveWindow.disableAllTelemetry();

			mSubsystemManager.setSubsystems(
					mDrive,
					mSuperstructure,
					mLimelight,
					mPivot,
					mElevator,
					mWrist,
					mEndEffector,
					mShooter,
					mClimberHook

			);

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			// mLoggingLooper.register(mLogger.Loop());

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void robotPeriodic() {
		mShuffleboard.update();
		mSubsystemManager.outputToSmartDashboard();
		mEnabledLooper.outputToSmartDashboard();
	}

	@Override
	public void autonomousInit() {

		try {
			mDisabledLooper.stop();
			Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
			if (autoMode.isPresent()) {
				mDrive.resetOdometry(autoMode.get().getStartingPose());
				System.out.println("ODOMETRY RESET FOR AUTO");
			}

			mEnabledLooper.start();
			mAutoModeExecutor.start();
			// mLoggingLooper.start();
			mControlBoard.setAutoSnapToTarget(false);

			mDrive.setNeutralBrake(true);

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
		CrashTracker.logAutoInit();

	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
		mDrive.orientModules(List.of(
				Rotation2d.fromDegrees(45),
				Rotation2d.fromDegrees(-45),
				Rotation2d.fromDegrees(-45),
				Rotation2d.fromDegrees(45)));
	}

	@Override
	public void teleopInit() {
		try {
			if (is_red_alliance) { // TODO: UNDO THIS IF THE ROBOT DOESNT START IN THE RIGHT ORIENTATION
				mDrive.zeroGyro(mDrive.getHeading().getDegrees() + 180.0);
				flip_trajectories = false;
			}
			mDisabledLooper.stop();
			mEnabledLooper.start();
			// mLoggingLooper.start();
			mSuperstructure.stop();

			mDrive.setAutoSpinFast(false);

			mControlBoard.setAutoSnapToTarget(false);

			mDrive.setNeutralBrake(true);
			mClimberHook.setWantNeutralBrake(true);
			mSuperstructure.disableAutoShot();

			mLimelight.setShootingFromMid2Piece(false);
			mLimelight.setShootingFromStage2Piece(false);
			mLimelight.setShootingFromAmp2Piece(false);
			mLimelight.setShootingSideOfSubwoofer(false);

			mSuperstructure.setSuperstuctureShoot(false); // prevents robot from catching note after 1st shot

			mSuperstructure.setManualControlMode(Constants.isManualControlMode);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopPeriodic() {
		try {

			/* Drive */
			if (mControlBoard.zeroGyro()) {
				mDrive.zeroGyro();
				mDrive.resetModulesToAbsolute();
			}

			if (mControlBoard.getBrake()) {
				mDrive.orientModules(List.of(
						Rotation2d.fromDegrees(45),
						Rotation2d.fromDegrees(-45),
						Rotation2d.fromDegrees(-45),
						Rotation2d.fromDegrees(45)));
			} else {
				mDrive.feedTeleopSetpoint(ChassisSpeeds.fromFieldRelativeSpeeds(
						mControlBoard.getSwerveTranslation().x(),
						mControlBoard.getSwerveTranslation().y(),
						mControlBoard.getSwerveRotation(),
						mDrive.getHeading()));

			}

			// if (mControlBoard.snapToTarget()) {
			// mDrive.setHeadingControlTarget(202.5);
			// mDrive.feedTeleopSetpoint(ChassisSpeeds.fromFieldRelativeSpeeds(0,
			// 0,
			// 0,
			// mDrive.getHeading()));
			// //shootFromPodiumBoolean = true;
			// //mDrive.setHeadingControlTarget(-mLimelight.getTargetSnap());
			// }
			// else if (!mControlBoard.snapToTarget()){
			// shootFromPodiumBoolean = false;
			// }

			/* Uncomment to enable podium shots */
			// if (mControlBoard.farLeftSwitchUp()) {
			// if (!is_red_alliance) {
			// mDrive.setHeadingControlTarget(202.5);
			// } else {
			// mDrive.setHeadingControlTarget(360 - 202.5);
			// }
			// mDrive.feedTeleopSetpoint(ChassisSpeeds.fromFieldRelativeSpeeds(0,
			// 0,
			// 0,
			// mDrive.getHeading()));
			// }

			// mLimelight.setShootingFromPodium(mControlBoard.farLeftSwitchUp());

			// if (mControlBoard.operator.getController().getRawButton(9)
			// && mControlBoard.operator.getController().getRawButton(10)) {
			// mElevator.setWantHome(true);
			// }

			if (mControlBoard.allignWithHumanPlayer()) {
				if (!is_red_alliance) { // keep in mind the alliance is flipped
					mDrive.setHeadingControlTarget(60);
				} else {
					mDrive.setHeadingControlTarget(-60);
				}
			} else if (mControlBoard.passNoteFromMidAllign()) {
				if (!is_red_alliance) { // keep in mind the alliance is flipped
					mDrive.setHeadingControlTarget(-140);
				} else {
					mDrive.setHeadingControlTarget(140);
				}
			} else if (mControlBoard.shootFromPodiumAllign()) {
				if (!is_red_alliance) { // keep in mind the alliance is flipped
					mDrive.setHeadingControlTarget(207);
				} else {
					mDrive.setHeadingControlTarget(-207);
				}
			} else if (mControlBoard.shootFromOppositePodiumAllign()) {
				// if (!is_red_alliance) { //keep in mind the alliance is flipped
				// mDrive.setHeadingControlTarget(-207);
				// } else {
				// mDrive.setHeadingControlTarget(207);
				// }
			} else if (mControlBoard.shootFromPodium() && (mControlBoard.farLeftSwitchUp()
					&& !Util.epsilonEquals(207, mDrive.getHeading().getDegrees(), 5))) { // extra check to make sure it
																							// stays the right angle
				if (!is_red_alliance) { // keep in mind the alliance is flipped
					mDrive.setHeadingControlTarget(207);
				} else {
					mDrive.setHeadingControlTarget(-207);
				}
			}

			// if (!mLimelight.cantFindTargetOnInitialSnap() &&
			// mControlBoard.snapToTarget()){
			// System.out.println("Snapping to target" + mLimelight.getTargetSnap());
			// mDrive.setHeadingControlTarget(mLimelight.getTargetSnap()); //only called
			// once per switch flip up
			// //autoAllignBoolean = true; //take out this to make always auto aim
			// }

			// if
			// (mControlBoard.snapToTarget()&&mLimelight.cantFindTargetOnInitialSnap()&&mLimelight.hasTarget()){
			// //makes it so when its spinning to 180 if it sees the target it will auto aim
			// instead of just going 180
			// mDrive.setHeadingControlTarget(mLimelight.getTargetSnap());
			// } //flip 180 if no tag seen, and continue searching for tag until tag seen
			// and then snap to correct angle (2 snaps total)

			// if (!mControlBoard.snapToTarget()){
			// autoAllignBoolean = false;
			// }

			if (Constants.isManualControlMode) {
				/* PIVOT */

				// if (Math.abs(mControlBoard.pivotPercentOutput()) > 0.1) {
				// mSuperstructure.controlPivotManually(mControlBoard.pivotPercentOutput());
				// } else if (mControlBoard.pivotUp()) {
				// mPivot.setSetpointMotionMagic(80);
				// } else if (mControlBoard.pivotDown()) {
				// mPivot.setSetpointMotionMagic(10);
				// }

				mSuperstructure.controlPivotManually(mControlBoard.operator.getController().getLeftY());

				/* ELEVATOR */

				// if (Math.abs(mControlBoard.elevatorPercentOutput())){
				mSuperstructure.controlElevatorManually(mControlBoard.elevatorPercentOutput());
				// }
				if (mControlBoard.operator.getButton(Button.RB)) {
					mSuperstructure.controlWristManually(1);
					// mElevator.setSetpointMotionMagic(0.4);
				} else if (mControlBoard.operator.getButton(Button.LB)) {
					mSuperstructure.controlWristManually(-1);
					// mElevator.setSetpointMotionMagic(0.00);
				}

				if (mControlBoard.operator.getController().getPOV() == kDpadLeft) {
					mClimberHook.setSetpointMotionMagic(Constants.ClimberHookConstants.kHookAngle);
					// mClimberHook.setDemandOpenLoop(0.2);
					// mSuperstructure.controlClimberHookManually(1);
					// mElevator.setSetpointMotionMagic(0.4);
				}
				if (mControlBoard.operator.getController().getPOV() == kDpadRight) {
					// mClimberHook.setDemandOpenLoop(0.0);
					mClimberHook.setSetpointMotionMagic(25);
					// mElevator.setSetpointMotionMagic(0.00);
				}

				/* WRIST */

				// if (Math.abs(mControlBoard.operator.getController().getLeftX())>0.1){
				// mSuperstructure.controlWristManually(mControlBoard.operator.getController().getLeftX());

				// }
				if (mControlBoard.operator.getButton(Button.X)) {
					mShooter.setOpenLoopDemand(0.95); // 6380 max
					// mWrist.setSetpointMotionMagic(10);
				} else {// if (mControlBoard.operator.getButton(Button.B)){
					mShooter.setOpenLoopDemand(0);
				}

				/* END EFFECTOR */

				// if (mControlBoard.operator.getController().getPOV() == kDpadUp){
				// mEndEffector.setEndEffectorVelocity(2000);
				// }
				// else if(mControlBoard.operator.getController().getPOV() == kDpadDown){
				// mEndEffector.setEndEffectorVelocity(0);
				// }

				// if (mControlBoard.operator.getTrigger(Side.RIGHT)) {
				// mEndEffector.setState(State.INTAKING);
				// } else if (mControlBoard.operator.getTrigger(Side.LEFT)) {
				// mEndEffector.setState(State.OUTTAKING);
				// } else {
				// mEndEffector.setState(State.IDLE);
				// }

			} else {
				/*
				 * Dpads:
				 * Down: intake ground
				 * Right: intake source (human player)
				 * Left: stow
				 * Up: outtake for amp or shoot when in shooting mode
				 * 
				 * Triggers:
				 * Left Trigger: Score Amp
				 * Right Trigger: Manual intake
				 * 
				 * CLIMB:
				 * press both Start+Back button to go to initial chain hook height
				 * press both bumpers to engage stage 2 climb which pulls the robot onto the
				 * chain
				 * press A to engage stage 3 climb which scores into the trap, press Dpad to
				 * eject note
				 */

				/*
				 * TODO:
				 * Manual Control of elevator during climb
				 * Do more climbing tests
				 * 
				 */
				if ((!mSuperstructure.inClimbMode()) && (!mSuperstructure.isDeclimbing())) {
					if (mControlBoard.operator.getTrigger(Side.LEFT)) {
						mSuperstructure.setSuperstuctureScoreAmp();
					} else if (mControlBoard.operator.getController().getPOV() == kDpadDown) {
						mSuperstructure.setSuperstuctureIntakingGround();
						// setSuperstuctureShoot();
					} else if (mControlBoard.operator.getController().getPOV() == kDpadRight) {
						mSuperstructure.setSuperstuctureIntakingSource();
					} else if (mControlBoard.operator.getController().getPOV() == kDpadLeft) {
						mSuperstructure.setSuperstuctureStow();
					} else if (mControlBoard.operator.getButton(Button.START)
							&& mControlBoard.operator.getButton(Button.BACK)) {
						mSuperstructure.setClimbMode();
					} else if (mControlBoard.operator.getTrigger(Side.RIGHT)) {
						mSuperstructure.setSuperstuctureTransferToShooter();
					} else if (mControlBoard.operator.getButton(Button.RB)) {
						mSuperstructure.setSuperstuctureSourceLoadShooter();
					} else if (mControlBoard.operator.getButton(Button.A)) {
						mSuperstructure.setSuperstuctureShooterToEndEffector();
					} else if (mControlBoard.operator.getButton(Button.LB) && mManualSourceBoolean == false) {
						mManualSourceIntakeTimer.reset();
						mManualSourceIntakeTimer.start();
						mManualSourceBoolean = true;
					} else if (!mControlBoard.operator.getButton(Button.LB)) {
						mManualSourceBoolean = false;
					} else if (mManualSourceIntakeTimer.get() > 1 && mControlBoard.operator.getButton(Button.LB)) {
						mSuperstructure.setManualSourceIntake();
					}

					if (mControlBoard.operator.getButton(Button.B)) { // used to be sending button b all the time, this
																		// makes it more automated but you can't undo --
																		// make sure to test this
						mSuperstructure.setSuperstuctureShoot(true);
					}

				}
				if (mSuperstructure.inClimbMode()) {
					if (mControlBoard.operator.getButton(Button.RB) && mControlBoard.operator.getButton(Button.LB)) {
						mSuperstructure.setClimbModeStage2();
					}
					if (mControlBoard.operator.getTrigger(Side.LEFT) && mControlBoard.operator.getTrigger(Side.RIGHT)) {
						mSuperstructure.setClimbModeStage3();
					}
					if (mSuperstructure.climbFinished() && mControlBoard.operator.getButton(Button.START)
							&& mControlBoard.operator.getButton(Button.BACK)) {
						mSuperstructure.setSuperstuctureDeclimb();
					}
				}

				if (mSuperstructure.isDeclimbing()) {
					// if (mControlBoard.operator.getButton(Button.X)) {
					// mSuperstructure.setDeClimbUnhook();
					// }
					// if (mControlBoard.operator.getButton(Button.Y)) {
					// mSuperstructure.setDeclimbWantsElevatorDown(); // doesn't do anything unless
					// in declimb mode
					// }
				}

				// mSuperstructure.setWantOuttake((mControlBoard.operator.getController().getPOV()
				// == kDpadUp));
				// mSuperstructure.setWantIntake(mControlBoard.operator.getTrigger(Side.RIGHT));

				// mShooter.setOpenLoopDemand(mControlBoard.operator.getController().getLeftY());
			}

			// if (mControlBoard.getSwerveSnap() != SwerveCardinal.NONE) {
			// mDrive.setHeadingControlTarget(mControlBoard.getSwerveSnap().degrees);
			// SmartDashboard.putNumber("Snapping Drgrees",
			// mControlBoard.getSwerveSnap().degrees);
			// } else {
			// SmartDashboard.putNumber("Snapping Drgrees", -1);
			// }

			// if ((mControlBoard.getWantChase()) && (!wantChase)) {
			// // mLimelight.setWantChase(true);
			// mSuperstructure.tagTrajectory(mControlBoard.tagToChase(),
			// mControlBoard.chaseNearest());
			// wantChase = true;
			// doneChasing = false;
			// }
			// if ((!mControlBoard.getWantChase()) && (wantChase)) {
			// mDrive.stopModules();
			// // mLimelight.setWantChase(false);
			// wantChase = false;
			// doneChasing = true;
			// } else {
			// // mLimelight.updatePoseWithLimelight();
			// }

			// SmartDashboard.putBoolean("Done with tag trajectory",
			// mSuperstructure.isFinishedWithTagTrajectory());

			// /* SUPERSTRUCTURE */

			// if (mControlBoard.driver.getTrigger(Side.RIGHT)) {
			// mSuperstructure.setEndEffectorForwards();
			// } else if ((mControlBoard.driver.getTrigger(Side.LEFT))) {
			// mSuperstructure.setEndEffectorReverse();
			// } else {
			// mSuperstructure.setEndEffectorIdle();
			// }

			// if (mControlBoard.driver.getController().getPOV() == 0) {
			// mWrist.setWantJog(1.0);
			// } else if (mControlBoard.driver.getController().getPOV() == 180) {
			// mWrist.setWantJog(-1.0);
			// }

			// if (mControlBoard.driver.getController().getLeftBumper()) {
			// mSuperstructure.stowState();
			// mDrive.setKinematicLimits(Constants.SwerveConstants.kUncappedLimits);
			// } else if (mControlBoard.driver.getController().getRightBumper()) {
			// mSuperstructure.chooseScoreState();
			// } else if (mControlBoard.operator.getController().getRightBumper()) {
			// mSuperstructure.groundIntakeState();
			// } else if (mControlBoard.operator.getButton(Button.A)) {
			// mSuperstructure.chooseShelfIntake();
			// } else if (mControlBoard.operator.getButton(Button.Y)) {
			// mSuperstructure.slideIntakeState();
			// }else if (mControlBoard.operator.getButton(Button.X)) {
			// mSuperstructure.scoreStandbyState();
			// } else if (mControlBoard.operator.getButton(Button.B)) {
			// mSuperstructure.yoshiState();
			// } else if
			// (mControlBoard.operator.getController().getLeftStickButtonPressed()) {
			// mSuperstructure.climbFloatState();
			// } else if
			// (mControlBoard.operator.getController().getRightStickButtonPressed()) {
			// mSuperstructure.climbScrapeState();
			// } else if (mControlBoard.operator.getTrigger(Side.LEFT)) {
			// mSuperstructure.climbCurlState();
			// }

			// if (mControlBoard.driver.getController().getLeftStickButton()) {
			// mDrive.setKinematicLimits(Constants.SwerveConstants.kScoringLimits);
			// } else if (mControlBoard.driver.getController().getLeftStickButtonReleased())
			// {
			// mDrive.setKinematicLimits(Constants.SwerveConstants.kUncappedLimits);
			// }

			// if (mControlBoard.driver.getController().getRightStickButton()) {
			// mDrive.setKinematicLimits(Constants.SwerveConstants.kLoadingStationLimits);
			// } else if
			// (mControlBoard.driver.getController().getRightStickButtonReleased()) {
			// mDrive.setKinematicLimits(Constants.SwerveConstants.kUncappedLimits);
			// }

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledInit() {
		try {

			CrashTracker.logDisabledInit();
			mEnabledLooper.stop();
			// mLoggingLooper.stop();
			mDisabledLooper.start();
			mClimberHook.setWantNeutralBrake(false);

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

		if (mAutoModeExecutor != null) {
			mAutoModeExecutor.stop();
		}

		// Reset all auto mode state.
		mAutoModeSelector.reset();
		mAutoModeSelector.updateModeCreator(false);
		mAutoModeExecutor = new AutoModeExecutor();

	}

	@Override
	public void disabledPeriodic() {
		try {

			mDrive.resetModulesToAbsolute();
			mPivot.resetToAbsolute();
			mWrist.resetToAbsolute();
			mElevator.zeroWhenDisabled();

			// mDrive.outputTelemetryDisabled();

			boolean alliance_changed = false;
			if (DriverStation.isDSAttached()) {
				Optional<Alliance> ally = DriverStation.getAlliance();
				if (ally.isPresent()) {
					if (ally.get() == Alliance.Red) {
						if (!is_red_alliance) {
							alliance_changed = true;
						} else {
							alliance_changed = false;
						}
						is_red_alliance = true;
					}
					if (ally.get() == Alliance.Blue) {
						if (is_red_alliance) {
							alliance_changed = true;
						} else {
							alliance_changed = false;
						}
						is_red_alliance = false;
					}
					is_red_alliance = !is_red_alliance; // TODO: this is only for red right side
				}
			} else {
				alliance_changed = true;
			}
			flip_trajectories = is_red_alliance; // TODO: THIS MIGHT MESS EVERYTHING UP?

			// SmartDashboard.putBoolean("is_red_alliance", is_red_alliance);
			mAutoModeSelector.updateModeCreator(alliance_changed);
			Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
			mLimelight.isRedAlliance(is_red_alliance);
			if (autoMode.isPresent()) {
				mAutoModeExecutor.setAutoMode(autoMode.get());
			}

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {
		try {
			mDisabledLooper.stop();
			mEnabledLooper.stop();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void simulationPeriodic() {
		// PhysicsSim.getInstance().run();
	}
}