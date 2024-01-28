// package com.team8013.frc2023.subsystems;

// import com.team8013.frc2023.Constants;
// import com.team8013.frc2023.RobotState;
// import com.team8013.frc2023.logger.LogStorage;
// import com.team8013.frc2023.logger.LoggingSystem;
// import com.team8013.frc2023.loops.ILooper;
// import com.team8013.frc2023.loops.Loop;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.Pair;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;

// import java.util.ArrayList;
// import java.util.List;
// import java.util.Optional;

// /**
//  * Subsystem for interacting with the Limelight 2
//  */
// public class Limelight extends Subsystem {
//   private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    
//   private final NetworkTableEntry tX = networkTable.getEntry("tx");
//   private final NetworkTableEntry tY = networkTable.getEntry("ty");
//   private final NetworkTableEntry tV = networkTable.getEntry("tv");
//   private final NetworkTableEntry tL = networkTable.getEntry("tl");
//   private final NetworkTableEntry tBotPose = networkTable.getEntry("botpose");
//   private final NetworkTableEntry tPipeline = networkTable.getEntry("pipeline");

//   private final Field2d field = new Field2d();

//   public Limelight() {
//     SmartDashboard.putData("Field2", field);
//   }
  
//   // Returns true if LL has target
//   public boolean hasTarget(){
//     return tV.getDouble(0.0) == 1.0;
//   }

//   public double getLatency() {
//     // 11 additional ms is recommended for image capture latency
//     return (tL.getDouble(0.0) + 11.0)/1000.0; //divided by 1000.0 to convert ms to s
//   }

//   public Pair<Pose2d, Double> getBotPose() {
//     // If LL does not have target return null
//     if (!hasTarget()) {
//       return null;
//     }

//     // Current time accounting for latency
//     double currentTime = Timer.getFPGATimestamp() - getLatency();

//     // LL pose array
//     double[] limelightBotPoseArray = tBotPose.getDoubleArray(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

//     // Ensuring the 
//     if (limelightBotPoseArray == null || limelightBotPoseArray.length < 6) {
//       return null;
//     }

//     Pose2d pose = new Pose3d(new Translation3d(limelightBotPoseArray[0], limelightBotPoseArray[1],limelightBotPoseArray[2]), new Rotation3d(Math.toRadians(limelightBotPoseArray[3]), Math.toRadians(limelightBotPoseArray[4]), Math.toRadians(limelightBotPoseArray[5]))).toPose2d();
    
//     if (pose == null){
//       return null;
//     } 

//     //transform pose from LL "field space" to pose2d
//     pose = new Pose2d(pose.getTranslation().plus(new Translation2d(Constants.FieldConstants.length/2.0, Constants.FieldConstants.width/2.0)), pose.getRotation());

//     // System.out.println("LL Field2d");
//     // System.out.println(pose);

//     field.setRobotPose(pose);
    
//     return new Pair<Pose2d, Double> (pose, currentTime);
//   }

//   // Setting periodic pipeline
//   @Override
//   public void periodic() {
//     tPipeline.setNumber(0);
//   }
// }