// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team8013.frc2024;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

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
import com.team8013.frc2024.subsystems.Limelight;
import com.team8013.frc2024.subsystems.Pivot;
import com.team8013.frc2024.subsystems.Shooter;
import com.team8013.frc2024.subsystems.Superstructure;
import com.team8013.frc2024.subsystems.Wrist;
import com.team8013.frc2024.subsystems.EndEffectorREV.State;
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
			if (is_red_alliance) {
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

				mSuperstructure.controlPivotManually(mControlBoard.operator.getController().getLeftY());

				/* ELEVATOR */

				mSuperstructure.controlElevatorManually(mControlBoard.elevatorPercentOutput());

				/* WRIST */
				if (mControlBoard.operator.getButton(Button.RB)) {
					mSuperstructure.controlWristManually(1);
				} else if (mControlBoard.operator.getButton(Button.LB)) {
					mSuperstructure.controlWristManually(-1);
				}

				/* CLIMBER HOOKS */
				if (mControlBoard.operator.getController().getPOV() == kDpadLeft) {
					mClimberHook.setSetpointMotionMagic(Constants.ClimberHookConstants.kHookAngle);
				}

				if (mControlBoard.operator.getController().getPOV() == kDpadRight) {
					mClimberHook.setSetpointMotionMagic(25);
				}

				if (mControlBoard.operator.getButton(Button.X)) {
					mShooter.setOpenLoopDemand(0.95);
				} else {
					mShooter.setOpenLoopDemand(0);
				}

				/* END EFFECTOR */

				if (mControlBoard.operator.getTrigger(Side.RIGHT)) {
					mEndEffector.setState(State.INTAKING);
				} else if (mControlBoard.operator.getTrigger(Side.LEFT)) {
					mEndEffector.setState(State.OUTTAKING);
				} else {
					mEndEffector.setState(State.IDLE);
				}

			} else {
				/*
				 * -- CONTROLS --
				 * Dpads:
				 * Down: intake ground
				 * Right: intake source (human player)
				 * Left: stow
				 * Up: outtake, for amp, trap, and low passing
				 * 
				 * Triggers:
				 * Left Trigger: Score Amp
				 * Right Trigger: Shoot/Load shooter and shoot, press B to shoot
				 * A Button: Pass note from shooter to end effector
				 * 
				 * Bumpers:
				 * Right bumper: Load shooter or go to source intake and immediatly load shooter when note detected
				 * Left bumper: if held for a second, robot goes into manual source intake mode which is an override if the beam breaks are out
				 * 
				 * CLIMB:
				 * press both Start+Back button to go to initial chain hook height
				 * press both bumpers to engage stage 2 climb which pulls the robot onto the
				 * chain
				 * press both triggers to engage stage 3 climb which scores into the trap
				 * press Dpad to
				 * eject note
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
					if (mSuperstructure.climbFinished() && mControlBoard.operator.getButton(Button.START)
							&& mControlBoard.operator.getButton(Button.BACK)) {
						mSuperstructure.setSuperstuctureDeclimb();
					}
				}
			}

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
					is_red_alliance = !is_red_alliance; // had to flip this because I drew the trajectories on the red
														// side
				}
			} else {
				alliance_changed = true;
			}
			flip_trajectories = is_red_alliance;

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