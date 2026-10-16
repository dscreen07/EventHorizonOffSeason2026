// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
  private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTable elasticTable = NetworkTableInstance.getDefault().getTable("Turret");
  private final DoublePublisher cameraTemp = elasticTable.getDoubleTopic("Limelight Temp").publish();
  private final BooleanPublisher hasTargetPub = elasticTable.getBooleanTopic("Has Target").publish();
  private final IntegerArrayPublisher tagIDs = elasticTable.getIntegerArrayTopic("Tag IDs").publish();
  private final CommandSwerveDrivetrain drivetrain;
  // Create a memory variable to track what we've already set it to
  private Alliance currentPipelineAlliance = null;

  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {

    this.drivetrain = drivetrain;
    // Configure the static 3D pose of the camera relative to the robot center
    LimelightHelpers.setCameraPose_RobotSpace(
          "limelight", 
          VisionConstants.kCameraForward, 
          VisionConstants.kCameraSide, 
          VisionConstants.kCameraUp, 
          VisionConstants.kCameraRoll, 
          VisionConstants.kCameraPitch, 
          VisionConstants.kCameraYaw
      );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Limelight Temp", getTemp());
    SmartDashboard.putBoolean("Has Target", hasTarget());

    double currentYaw = drivetrain.getState().Pose.getRotation().getDegrees();

    LimelightHelpers.SetRobotOrientation("limelight", currentYaw, 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    if((mt2Estimate != null) && (mt2Estimate.tagCount > 0)){
      double distance = mt2Estimate.avgTagDist;
      double xyTrust = 0.5 + (0.3 * distance);
      //Failsafe: Ignore very high-distance tags
      if(mt2Estimate.tagCount == 1 && mt2Estimate.avgTagDist > 4.0){
        return;
      }

      drivetrain.addVisionMeasurement(
        mt2Estimate.pose, 
        mt2Estimate.timestampSeconds, 
        VecBuilder.fill(xyTrust, xyTrust, 9999999)
        );
    }
    
    updateVisibleTagIDs();
    // updateLimelightPipeline();
  }

  // public void updateLimelightPipeline() {
  //   Optional<Alliance> ally = DriverStation.getAlliance();

  //   // Did the Driver Station give us a color?
  //   if (ally.isPresent()) {
  //     Alliance realAlliance = ally.get();

  //     // Is this color DIFFERENT than the one we already set?
  //     if (realAlliance != currentPipelineAlliance) {

  //       // Update the Limelight!
  //       if (ally.get() == Alliance.Blue) {
  //         limelightTable.getEntry("pipeline").setNumber(1);
  //       } else if (ally.get() == Alliance.Red) {
  //         limelightTable.getEntry("pipeline").setNumber(0);
  //       }

  //       // Save the new color to memory so we don't spam it again
  //       currentPipelineAlliance = realAlliance;
  //     }
  //   }
  // }

  public int getTemp() {
    double[] hw = limelightTable.getEntry("hw").getDoubleArray(new double[]{0,0,0,0});
    return (int) hw[0];
  }

  public boolean hasTarget(){
    return limelightTable.getEntry("tv").getDouble(0) == 1.0;
  }

  public void updateVisibleTagIDs() {
    double[] rawIDs = limelightTable.getEntry("tid").getDoubleArray(new double[0]);

    long[] parsedIDs = new long[rawIDs.length];
    for (int i = 0; i < rawIDs.length; i++){
      parsedIDs[i] = (long) rawIDs[i];
    }
    tagIDs.set(parsedIDs);
  }
}