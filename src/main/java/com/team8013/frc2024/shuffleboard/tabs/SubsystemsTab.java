package com.team8013.frc2024.shuffleboard.tabs;

import com.team8013.frc2024.shuffleboard.ShuffleboardTabBase;
import com.team8013.frc2024.subsystems.ClimberHook;
import com.team8013.frc2024.subsystems.Elevator;
import com.team8013.frc2024.subsystems.EndEffectorREV;
import com.team8013.frc2024.subsystems.Pivot;
import com.team8013.frc2024.subsystems.Shooter;
import com.team8013.frc2024.subsystems.Wrist;
import com.team8013.lib.Conversions;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class SubsystemsTab extends ShuffleboardTabBase {

    /*
     * Information:
     * Pivot:
     *  CanCoder
     *  Motor Angle Calculated
     *  Motor Torque Current
     * 
     * Elevator:
     *  Elevator Position Meters
     *  Elevator Position Inches
     *  Motor Torque Current
     *  Elevator Demand ?
     * 
     * End Effector
     *  Velocity Top (when stowed)
     *  Velocity Bottom
     *  Beam Break
     *  Demand
     * 
     * Wrist:
     *  CanCoder
     *  Motor Angle Calculated
     *  Motor Torque Current
     * 
     * Climber Hooks:
     *  Angle
     * 
     * Shooter:
     *  Beam Break
     *  Demand
     * 
     * 
     */

    private Pivot mPivot = Pivot.getInstance();
    private Elevator mElevator = Elevator.getInstance();
    private EndEffectorREV mEndEffector = EndEffectorREV.getInstance();
    private Wrist mWrist = Wrist.getInstance();
    private ClimberHook mClimberHook = ClimberHook.getInstance();
    private Shooter mShooter = Shooter.getInstance();

    /*Pivot */
    private GenericEntry mPivotCanCoder;
    private GenericEntry mPivotMotorAngle;
    private GenericEntry mPivotTorqueCurrent;
    private ShuffleboardLayout mPivotLayout;

    /*Elevator */
    private GenericEntry mElevatorPositionInches;
    private GenericEntry mElevatorPositionMeters;
    private GenericEntry mElevatorTorqueCurrent;
    private GenericEntry mElevatorDemand;
    private ShuffleboardLayout mElevatorLayout;

    /*End Effector */
    private GenericEntry mEndEffectorVelocityTop;
    private GenericEntry mEndEffectorVelocityBottom;
    private GenericEntry mEndEffectorBeamBreak;
    private GenericEntry mEndEffectorDemand;
    private ShuffleboardLayout mEndEffectorLayout;

    /*Wrist */
    private GenericEntry mWristCanCoder;
    private GenericEntry mWristMotorAngle;
    private GenericEntry mWristTorqueCurrent;
    private ShuffleboardLayout mWristLayout;

    /*Climber Hook */
    private GenericEntry mClimberHookMotorAngle;
    private GenericEntry mClimberHookTorqueCurrent;
    private ShuffleboardLayout mClimberHookLayout;

    /*Shooter */
    private GenericEntry mShooterBeamBreak;
    private GenericEntry mShooterDemand;
    private ShuffleboardLayout mShooterLayout;

    public SubsystemsTab() {
        super();

    }

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Swerve");

        /*Pivot */
        mPivotLayout = mTab
            .getLayout("Pivot", BuiltInLayouts.kGrid)
            .withSize(2, 2)
            .withPosition(0 * 2, 0);
        mPivotCanCoder = mPivotLayout.add("Cancoder", 0.0)
            .withPosition(0, 0)
            .withSize(5, 1).getEntry();
        mPivotMotorAngle = mPivotLayout.add("Estimated Angle", 0)
            .withPosition(1, 0)
            .withSize(5, 1).getEntry();          
        mPivotTorqueCurrent = mPivotLayout.add("Torque Current", 0.0)
            .withPosition(0, 1)
            .withSize(5, 1).getEntry();


        /*Elevator */
        mElevatorLayout = mTab
        .getLayout("Elevator", BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(1 * 2, 0);
        mElevatorPositionInches = mElevatorLayout.add("Position Inches", 0.0)
        .withPosition(0, 0)
        .withSize(5, 1).getEntry();
        mElevatorPositionMeters = mElevatorLayout.add("Position Meters", 0.0)
        .withPosition(1, 0)
        .withSize(5, 1).getEntry();          
        mElevatorTorqueCurrent = mElevatorLayout.add("Torque Current", 0.0)
        .withPosition(0, 1)
        .withSize(5, 1).getEntry();
        mElevatorDemand = mElevatorLayout.add("Demand",0.0)
            .withPosition(1, 1)
            .withSize(5, 1)
            .getEntry();

        /*End Effector */
        mEndEffectorLayout = mTab
        .getLayout("Elevator", BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(2 * 2, 0);
        mEndEffectorVelocityTop = mEndEffectorLayout.add("Vel Top (stowed)", 0.0)
        .withPosition(0, 0)
        .withSize(5, 1).getEntry();
        mEndEffectorVelocityBottom = mEndEffectorLayout.add("Vel Bottom", 0.0)
        .withPosition(1, 0)
        .withSize(5, 1).getEntry();          
        mEndEffectorBeamBreak = mEndEffectorLayout.add("Beam Break", false)
        .withPosition(0, 1)
        .withSize(5, 1).getEntry();
        mEndEffectorDemand = mEndEffectorLayout.add("Demand",0.0)
        .withPosition(1, 1)
        .withSize(5, 1)
        .getEntry();

        /*Wrist */
        mWristLayout = mTab
        .getLayout("Wrist", BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(3 * 2, 0);
        mWristCanCoder = mWristLayout.add("Cancoder", 0.0)
        .withPosition(0, 0)
        .withSize(5, 1).getEntry();
        mWristMotorAngle = mWristLayout.add("Estimated Angle", 0)
        .withPosition(1, 0)
        .withSize(5, 1).getEntry();          
        mWristTorqueCurrent = mWristLayout.add("Torque Current", 0.0)
        .withPosition(0, 1)
        .withSize(5, 1).getEntry();

        /*Climber Hook */
        mClimberHookLayout = mTab
        .getLayout("Climber Hook", BuiltInLayouts.kGrid)
        .withSize(2, 1)
        .withPosition(0 * 2, 3);
        mClimberHookMotorAngle = mClimberHookLayout.add("Estimated Angle", 0.0)
        .withPosition(0, 0)
        .withSize(5, 1).getEntry();        
        mClimberHookTorqueCurrent = mClimberHookLayout.add("Torque Current", 0.0)
        .withPosition(0, 1)
        .withSize(5, 1).getEntry();

        /*Shooter */
        mShooterLayout = mTab
        .getLayout("Shooter", BuiltInLayouts.kGrid)
        .withSize(2, 1)
        .withPosition(1 * 2, 3);
        mShooterBeamBreak = mShooterLayout.add("Beam Break", 0.0)
        .withPosition(0, 0)
        .withSize(5, 1).getEntry();        
        mShooterDemand = mShooterLayout.add("Demand", 0.0)
        .withPosition(0, 1)
        .withSize(5, 1).getEntry();

    }

    @Override
    public void update() {
        
        /*Pivot */
        mPivotCanCoder.setDouble(truncate(mPivot.getCanCoder()));
        mPivotMotorAngle.setDouble(truncate(mPivot.getPivotAngleDeg()));
        mPivotTorqueCurrent.setDouble(truncate(mPivot.getPivotCurrent()));
        
        /*Elevator */
        mElevatorDemand.setDouble(truncate(mElevator.getElevatorDemand()));
        mElevatorPositionInches.setDouble(truncate(Conversions.metersToInches(mElevator.getElevatorUnits())));
        mElevatorPositionMeters.setDouble(truncate(mElevator.getElevatorUnits()));
        mElevatorTorqueCurrent.setDouble(truncate(mElevator.getElevatorCurrent()));

        /*End Effector */
        mEndEffectorBeamBreak.setBoolean(mEndEffector.hasGamePiece());
        mEndEffectorVelocityTop.setDouble(truncate(mEndEffector.getVelocityMaster()));
        mEndEffectorVelocityBottom.setDouble(truncate(mEndEffector.getVelocitySlave()));
        mEndEffectorDemand.setDouble(truncate(mEndEffector.getEndEffectorMasterDemand()));

        /*Wrist */
        mWristCanCoder.setDouble(truncate(mWrist.getCanCoder()));
        mWristMotorAngle.setDouble(truncate(mWrist.getWristAngleDeg()));
        mWristTorqueCurrent.setDouble(truncate(mWrist.getWristCurrent()));

        /*Climber Hooks */
        mClimberHookMotorAngle.setDouble(truncate(mClimberHook.getAngleDeg()));
        mClimberHookTorqueCurrent.setDouble(truncate(mClimberHook.getTorqueCurrent()));
        
        /*Shooter */
        mShooterBeamBreak.setBoolean(mShooter.getBeamBreak());
        mShooterDemand.setDouble(truncate(mShooter.getShooterDemand()));
    }

}