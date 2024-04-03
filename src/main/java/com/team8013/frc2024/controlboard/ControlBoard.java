package com.team8013.frc2024.controlboard;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team8013.frc2024.Constants;
import com.team8013.frc2024.Ports;
import com.team8013.frc2024.controlboard.CustomXboxController.Axis;
import com.team8013.frc2024.controlboard.CustomXboxController.Button;
import com.team8013.frc2024.controlboard.CustomXboxController.Side;
import com.team8013.frc2024.subsystems.Drive;
import com.team8013.lib.Util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlBoard {
    private final double kSwerveDeadband = Constants.stickDeadband;

    private final int kDpadUp = 0;
    private final int kDpadRight = 90;
    private final int kDpadDown = 180;
    private final int kDpadLeft = 270;

    private boolean leftBumperBoolean = false;

    private boolean leftSwitchReset = true;
    private boolean autoSnap = false;

    private static ControlBoard mInstance = null;

    int tagLastChased = -1;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private final GenericHID m_driver;
    public final CustomXboxController operator;

    private ControlBoard() {
        m_driver = new GenericHID(Ports.DRIVER_PORT);
        operator = new CustomXboxController(Ports.OPERATOR_PORT);
        leftBumperBoolean = m_driver.getRawButton(2);
    }

    
    public void setOperatorRumble(boolean on) {
        operator.setRumble(on);
    }

    /* DRIVER METHODS */
    public Translation2d getSwerveTranslation() {
        double forwardAxis = 0;
        double strafeAxis = 0;
        if (!Constants.ControllerConstants.isMamboController){ //NOT MAMBO
            forwardAxis = getRightThrottle();
            strafeAxis = getRightYaw();
        }
        else{
            forwardAxis = m_driver.getRawAxis(2);
            strafeAxis = m_driver.getRawAxis(1);
        }

        SmartDashboard.putNumber("Raw Y", forwardAxis);
        SmartDashboard.putNumber("Raw X", strafeAxis);

        forwardAxis = Constants.SwerveConstants.invertYAxis ? forwardAxis : -forwardAxis;
        strafeAxis = Constants.SwerveConstants.invertXAxis ? strafeAxis : -strafeAxis;


        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.norm()) < kSwerveDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(tAxes.x(), tAxes.y(), true);
            Translation2d deadband_vector = Translation2d.fromPolar(deadband_direction, kSwerveDeadband);

            double scaled_x = Util.scaledDeadband(forwardAxis, 1.0, Math.abs(deadband_vector.x()));
            double scaled_y = Util.scaledDeadband(strafeAxis, 1.0, Math.abs(deadband_vector.y()));
            return new Translation2d(scaled_x, scaled_y).scale(Drive.getInstance().getKinematicLimits().kMaxDriveVelocity);
        }
        

    }

    public double getSwerveRotation() {
        double rotAxis = 0;
        if (!Constants.ControllerConstants.isMamboController){
            rotAxis = getLeftYaw();
        }
        else{
            rotAxis = m_driver.getRawAxis(3);
        }
        
        rotAxis = Constants.SwerveConstants.invertRAxis ? rotAxis : -rotAxis;
    
            if (Math.abs(rotAxis) < kSwerveDeadband) {
                return 0.0;
            } else {
                return Drive.getInstance().getKinematicLimits().kMaxAngularVelocity * (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband))
                        / (1 - kSwerveDeadband);
            }
    }

    /*right switch up */
    public boolean zeroGyro() {
        //SmartDashboard.putBoolean("Zero Gyro", m_driver.getRawButton(2));
        return m_driver.getRawButton(2);//(m_driver.getRawAxis(6)<-0.3);
    }
    
        public enum SwerveCardinal {
        NONE(0),

        FORWARDS(0),
        LEFT(270),
        RIGHT(90),
        BACKWARDS(180);

        public final double degrees;

        SwerveCardinal(double degrees) {
            this.degrees = degrees;
        }
    }

    public SwerveCardinal getSwerveSnap() {
        // CARDINAL SNAPS

        switch (operator.getController().getPOV()) {
            case kDpadUp:
                return SwerveCardinal.FORWARDS;
            case kDpadLeft:
                return SwerveCardinal.RIGHT;
            case kDpadRight:
                return SwerveCardinal.LEFT;
            case kDpadDown:
                return SwerveCardinal.BACKWARDS;
            default:
                return SwerveCardinal.NONE;
        }
            
    }

    /**far left switch */
    public boolean snapToTarget(){ //DISABLED
        return false;//m_driver.getRawAxis(4)<-0.25 || autoSnap;


        // if (m_driver.getRawAxis(4)<-0.25 && leftSwitchReset){
        //     leftSwitchReset = false;
        //     return true;
        // }
        // else if(!(m_driver.getRawAxis(4)<-0.25)){
        //     leftSwitchReset = true;
        // }
        //return false;
    }

    public void setAutoSnapToTarget(boolean snap){
        autoSnap = snap;
    }

    //public boolean farLeftSwitchUp(){//DISABLED
    //     return false; //m_driver.getRawAxis(4)<-0.25;
    // }

    /**right bumper */
    public boolean allignWithHumanPlayer(){
        if (leftBumperBoolean!=m_driver.getRawButton(1)){
            leftBumperBoolean = !leftBumperBoolean;
            return true;
        }

        return false;
        //;
    }

    public double pivotPercentOutput(){
        return operator.getAxis(Side.LEFT, Axis.Y);
    }


    public boolean pivotUp(){
        return operator.getButton(Button.Y);
    }

    public boolean pivotDown(){
        return operator.getButton(Button.A);
    }
    
    
    public boolean elevatorUp(){
        return operator.getButton(Button.RB);
    }

    public boolean elevatorDown(){
        return operator.getButton(Button.LB);
    }

    public boolean zeroElevator(){
        return operator.getButton(Button.START);
    }

    public double elevatorPercentOutput(){
        return operator.getAxis(Side.RIGHT, Axis.Y);
    }

    public double wristPercentOutput(){
        return operator.getAxis(Side.RIGHT, Axis.X);
    }

    public boolean endEffectorIntake(){
        return operator.getButton(Button.Y);
    }
    public boolean endEffectorOuttake(){
        return operator.getButton(Button.A);
    }


    // Align swerve drive with target
    public boolean getWantChase() {
        return false;//(m_driver.getRawButton(9)||(operator.getButton(Button.Y))||(operator.getButton(Button.B))||(operator.getButton(Button.A))||(operator.getButton(Button.X)));
    }

    public int tagToChase(){
        if (chaseTag1()){
            tagLastChased = 1;
            return 1;
        }
        else if (chaseTag2()){
            tagLastChased = 2;
            return 2;
        }
        else if (chaseTag3()){
            tagLastChased = 3;
            return 3;
        }
        else if (chaseTag4()){
            tagLastChased = 4;
            return 4;
        }
        else{
            return tagLastChased;
        }
    }

    public boolean chaseNearest(){
        return m_driver.getRawButton(9);
    }

    public boolean chaseTag1(){
        return (operator.getButton(Button.Y));
    }

    public boolean chaseTag2(){
        return (operator.getButton(Button.B));
    }
    
    public boolean chaseTag3(){
        return (operator.getButton(Button.A));
    }

    public boolean chaseTag4(){
        return (operator.getButton(Button.X));
    }


    // // Locks wheels in X formation
    public boolean getBrake() {
        SmartDashboard.putNumber("Get Brake", m_driver.getRawAxis(4));
        return  false;//(m_driver.getRawAxis(4)<-0.3); //m_driver.getRawButton(4); //far left switch
    }

    // // Intake Controls
    // public boolean getIntake() {
    //     return operator.getTrigger(CustomXboxController.Side.RIGHT);
    // }

    // public boolean getReject() {
    //     return operator.getTrigger(CustomXboxController.Side.LEFT);
    // }
    
    
    //Returns positions from -1 to 1 
    private double getLeftYaw() {
        double leftYaw = m_driver.getRawAxis(Constants.leftXAxis);

        if (leftYaw != 0){
            leftYaw = leftYaw - Constants.ControllerConstants.ControllerLeftYawZero;
        }

        if (leftYaw > kSwerveDeadband){
            leftYaw = (leftYaw / (Constants.ControllerConstants.ControllerLeftYawHigh + (Constants.ControllerConstants.isControllerOne ? -Constants.ControllerConstants.ControllerLeftYawZero : Constants.ControllerConstants.ControllerLeftYawZero)));
        } 
        else if (leftYaw < -kSwerveDeadband){
            leftYaw = (leftYaw / (Constants.ControllerConstants.ControllerLeftYawLow + Constants.ControllerConstants.ControllerLeftYawZero));
        }

        if (leftYaw>1){
            leftYaw = 1;
        }
        
        if (leftYaw<-1){
            leftYaw = -1;
        }

        //SmartDashboard.putNumber("remote leftYaw", leftYaw);
        return leftYaw;
    }

    //Returns positions from -1 to 1 
    // private double getLeftThrottle() {
    //     double leftThrottle = m_driver.getRawAxis(Constants.leftYAxis);

    //     if (leftThrottle != 0){
    //         leftThrottle = leftThrottle - Constants.ControllerConstants.ControllerLeftThrottleZero;
    //     }

    //     if (leftThrottle > kSwerveDeadband){
    //         leftThrottle = (leftThrottle / (Constants.ControllerConstants.ControllerLeftThrottleHigh + Constants.ControllerConstants.ControllerLeftThrottleZero));
    //     } 
    //     else if (leftThrottle < -kSwerveDeadband){
    //         leftThrottle = (leftThrottle / (Constants.ControllerConstants.ControllerLeftThrottleLow + Constants.ControllerConstants.ControllerLeftThrottleZero));
    //     }

    //     if (leftThrottle>1){
    //         leftThrottle = 1;
    //     }
        
    //     if (leftThrottle<-1){
    //         leftThrottle = -1;
    //     }

    //     //SmartDashboard.putNumber("remote leftThrottle", leftThrottle);
    //     return leftThrottle;
    // }

    private double getRightThrottle() {
        double rightThrottle = m_driver.getRawAxis(Constants.rightYAxis);

        if (rightThrottle != 0){
            rightThrottle = rightThrottle - Constants.ControllerConstants.ControllerRightThrottleZero;
        }

        if (rightThrottle > (Constants.ControllerConstants.isControllerOne ? kSwerveDeadband : 0.102)){
            rightThrottle = (rightThrottle / (Constants.ControllerConstants.ControllerRightThrottleHigh + (Constants.ControllerConstants.isControllerOne ? -Constants.ControllerConstants.ControllerRightThrottleZero : Constants.ControllerConstants.ControllerRightThrottleZero)));
        } 
        else if (rightThrottle < -kSwerveDeadband){
            rightThrottle = (rightThrottle / (Constants.ControllerConstants.ControllerRightThrottleLow + Constants.ControllerConstants.ControllerRightThrottleZero));
        }

        if (rightThrottle>1){
            rightThrottle = 1;
        }
        
        if (rightThrottle<-1){
            rightThrottle = -1;
        }

        //SmartDashboard.putNumber("remote rightThrottle", rightThrottle);
        return rightThrottle;
    }

    private double getRightYaw() {
        double rightYaw = m_driver.getRawAxis(Constants.rightXAxis);

        if (rightYaw != 0){
            rightYaw = rightYaw - Constants.ControllerConstants.ControllerRightYawZero;
        }

        if (rightYaw > kSwerveDeadband){
            rightYaw = (rightYaw / (Constants.ControllerConstants.ControllerRightYawHigh + -Constants.ControllerConstants.ControllerRightYawZero));
        } 
        else if (rightYaw < -kSwerveDeadband){
            rightYaw = (rightYaw / (Constants.ControllerConstants.ControllerRightYawLow + Constants.ControllerConstants.ControllerRightYawZero));
        }

        if (rightYaw>1){
            rightYaw = 1;
        }
        
        if (rightYaw<-1){
            rightYaw = -1;
        }

        //SmartDashboard.putNumber("remote rightYaw", rightYaw);
        return rightYaw;
    }
}

