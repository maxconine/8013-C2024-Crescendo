// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team8013.frc2024;

import java.io.IOException;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	public static boolean wantChase = false;
	public static boolean doneChasing = true;

	private final int kDpadUp = 0;
	private final int kDpadRight = 90;
	private final int kDpadDown = 180;
	private final int kDpadLeft = 270;

	public Robot() {
		CrashTracker.logRobotConstruction();
	}

	/* Credit to Team 2910 for this MAC Address based robot switching */
	static {
		List<byte[]> macAddresses;
		try {
			macAddresses = getMacAddresses();
		} catch (IOException e) {
			System.out.println("Mac Address attempt unsuccessful");
			System.out.println(e);
			macAddresses = List.of();
		}

		for (byte[] macAddress : macAddresses) {
			// first check if we are comp
			if (Arrays.compare(Constants.MacAddressConstants.COMP_ADDRESS, macAddress) == 0) {
				Constants.isComp = true;
				break;
			}
			// next check if we are beta
			else if (Arrays.compare(Constants.MacAddressConstants.BETA_ADDRESS, macAddress) == 0) {
				Constants.isBeta = true;
				break;
			}
			// if neither is true
			else {
				Constants.isComp = false;
				Constants.isBeta = false;
				System.out.println("New Mac Address Discovered!");
			}
		}

		if (!Constants.isComp && !Constants.isBeta) {
			// array
			String[] macAddressStrings = macAddresses.stream()
					.map(Robot::macToString)
					.toArray(String[]::new);

			SmartDashboard.putStringArray("MAC Addresses", macAddressStrings);
			// adds MAC addresses to the dashboard
			SmartDashboard.putString("Comp MAC Address", macToString(Constants.MacAddressConstants.COMP_ADDRESS));
			SmartDashboard.putString("Beta MAC Address", macToString(Constants.MacAddressConstants.BETA_ADDRESS));

			// if mac address doesn't work at comp
			Constants.isComp = true;
		}

		SmartDashboard.putBoolean("Comp Bot", Constants.isComp);
		SmartDashboard.putBoolean("Beta Bot", Constants.isBeta);

	}

	private static List<byte[]> getMacAddresses() throws IOException {
		List<byte[]> macAddresses = new ArrayList<>();

		Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();
		// connect to network
		NetworkInterface networkInterface;
		while (networkInterfaces.hasMoreElements()) {
			networkInterface = networkInterfaces.nextElement();

			byte[] address = networkInterface.getHardwareAddress();
			if (address == null) {
				continue;
			}

			macAddresses.add(address);
		}
		return macAddresses;
	}

	private static String macToString(byte[] address) {
		// changes string characters
		StringBuilder builder = new StringBuilder();
		for (int i = 0; i < address.length; i++) {
			if (i != 0) {
				builder.append(':');
			}
			builder.append(String.format("%02X", address[i]));
		}
		return builder.toString();
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
			}

			mEnabledLooper.start();
			mAutoModeExecutor.start();
			// mLoggingLooper.start();

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

			mDrive.setNeutralBrake(true);

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

			if (mControlBoard.operator.getController().getRawButton(9)
					&& mControlBoard.operator.getController().getRawButton(10)) {
				mElevator.setWantHome(true);
			}

			mSuperstructure.setManualControlMode(Constants.isManualControlMode);
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
					mSuperstructure.controlClimberHookManually(1);
					// mElevator.setSetpointMotionMagic(0.4);
				} else if (mControlBoard.operator.getController().getPOV() == kDpadRight) {
					mSuperstructure.controlClimberHookManually(-1);
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

				if (mControlBoard.operator.getTrigger(Side.RIGHT)) {
					mEndEffector.setState(State.INTAKING);
				} else if (mControlBoard.operator.getTrigger(Side.LEFT)) {
					mEndEffector.setState(State.OUTTAKING);
				} else {
					mEndEffector.setState(State.IDLE);
				}

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
				if (!mSuperstructure.inClimbMode()) {
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
					} else if (mControlBoard.operator.getButton(Button.RB)) {
						mSuperstructure.setSuperstuctureTransferToShooter();
					}


				} else {
					if (mControlBoard.operator.getButton(Button.RB) && mControlBoard.operator.getButton(Button.LB)) {
						mSuperstructure.setClimbModeStage2();
					}
					if (mControlBoard.operator.getButton(Button.A)) {
						mSuperstructure.setClimbModeStage3();
					}
				}

				mSuperstructure.setWantOuttake((mControlBoard.operator.getController().getPOV() == kDpadUp));
				mSuperstructure.setWantIntake(mControlBoard.operator.getTrigger(Side.RIGHT));

				//mShooter.setOpenLoopDemand(mControlBoard.operator.getController().getLeftY());
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
				}
			} else {
				alliance_changed = true;
			}
			flip_trajectories = is_red_alliance;

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