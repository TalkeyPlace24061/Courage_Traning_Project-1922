// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.attribute.standard.Fidelity;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;

public class LimeLight extends SubsystemBase {
  double tx;
  double ty;
  double ta;
  double txnc;
  double tync;
  boolean hasTarget;
  /** Creates a new LimeLight. */
  public LimeLight() {
    
      }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
     tx = LimelightHelpers.getTX("limelight-right"); // Horizontal offset from crosshar to target in degrees
     ty = LimelightHelpers.getTY("limelight-right"); // Vertical offset from crosshair to target in degrees
     ta = LimelightHelpers.getTA("limelight-right"); // Target area (0% to 100% of image)
     hasTarget = LimelightHelpers.getTV("limelight-right"); // Do you have a valid target

     txnc = LimelightHelpers.getTXNC("limelight-right"); // Horizontal offset from principal pixel/point to target in degrees 
     tync = LimelightHelpers.getTYNC("limelight-right"); // Vertical offset from principal pixel/point to target in degrees
    
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight-right");
    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id;
      txnc = fiducial.txnc;
      tync = fiducial.tync;
      ta = fiducial.ta;
      double distToCamera = fiducial.distToCamera;
      double distToRobot = fiducial.distToRobot;
      double ambiguity = fiducial.ambiguity;
      SmartDashboard.putNumber("Distance Meters", distToCamera);
    }
  }
    
    public double distToTarget(int target){
      RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight-right");
    for (RawFiducial fiducial : fiducials) {      
      int id = fiducial.id;

      if(target == id) {
        return fiducial.distToCamera;
      }      
    }
    return 0;
  }
}
