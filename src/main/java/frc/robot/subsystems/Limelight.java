// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;


public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public Limelight() {
     LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
   if(limelightMeasurement.tagCount >= 2)
   {
     LimelightHelpers.PoseEstimate.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
     LimelightHelpers.PoseEstimate.addVisionMeasurement(
         limelightMeasurement.pose,
         limelightMeasurement.timestampSeconds);
   }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Position: ", LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("Limelight").toString());
  }
}
