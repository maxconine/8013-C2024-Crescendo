package com.team8013.frc2024.shuffleboard.tabs;

import com.team8013.frc2024.shuffleboard.ShuffleboardTabBase;
import com.team8013.frc2024.subsystems.Limelight;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class VisionTab extends ShuffleboardTabBase {

	private Limelight mLimelight = Limelight.getInstance();

	private GenericEntry mSeesTarget;
	private GenericEntry mLimelightOk;
	private GenericEntry mLimelightLatency;
	private GenericEntry mLimelightTanToSpeaker;
	private GenericEntry mLimelightPosex, mLimelightPosexSmooth;
	private GenericEntry mLimelightPosey, mLimelightPoseySmooth;

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Vision");

		mLimelightOk = mTab
				.add("Limelight OK", false)
				.withPosition(0, 0)
				.withSize(1, 1)
				.getEntry();
		mSeesTarget = mTab
				.add("Limelight Sees Target", false)
				.withPosition(1, 0)
				.withSize(1, 1)
				.getEntry();
		mLimelightLatency = mTab
				.add("Limelight Latency", -1.0)
				.withPosition(2, 0)
				.withSize(2, 2)
				.withWidget(BuiltInWidgets.kTextView)
				.getEntry();
		mLimelightTanToSpeaker = mTab
				.add("Limelight Tan Line To Speaker", -1.0)
				.withPosition(4, 0)
				.withSize(2, 2)
				.withWidget(BuiltInWidgets.kTextView)
				.getEntry();
		mLimelightPosex = mTab
				.add("Limelight Bot Pose X", 0.0)
				.withPosition(0, 1)
				.withSize(1, 1)
				.getEntry();
		mLimelightPosey = mTab
				.add("Limelight Bot Pose Y", 0.0)
				.withPosition(1, 1)
				.withSize(1, 1)
				.getEntry();
		mLimelightPosexSmooth = mTab
				.add("Limelight Bot Pose X Smooth", 0.0)
				.withPosition(2, 1)
				.withSize(1, 1)
				.getEntry();
		mLimelightPoseySmooth = mTab
				.add("Limelight Bot Pose Y Smooth", 0.0)
				.withPosition(3, 1)
				.withSize(1, 1)
				.getEntry();
	}

	@Override
	public void update() {
		mSeesTarget.setBoolean(mLimelight.hasTarget());
		mLimelightOk.setBoolean(mLimelight.limelightOK());
		mLimelightLatency.setDouble(mLimelight.getLatency());
		mLimelightTanToSpeaker.setDouble(mLimelight.getTanLineToSpeaker());
		mLimelightPosex.setDouble(mLimelight.limelightBotPose2d().getX());
		mLimelightPosey.setDouble(mLimelight.limelightBotPose2d().getY());
		mLimelightPosexSmooth.setDouble(mLimelight.limelightBotPose2dSmooth().getX());
		mLimelightPoseySmooth.setDouble(mLimelight.limelightBotPose2dSmooth().getY());
		// mLimelightDt.setDouble(mLimelight.getDt());
		// mLimelightTx.setDouble(mLimelight.getOffset()[0]);
		// mLimelightTy.setDouble(mLimelight.getOffset()[1]);
	}

}
