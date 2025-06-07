// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TimeOfFlight extends SubsystemBase {
  LaserCan TOF = new LaserCan(4);
  /** Creates a new TOF. */
  public TimeOfFlight() {}

public int Distance(){
  LaserCan.Measurement measurement = TOF.getMeasurement();
  System.out.println("The target is " + measurement.distance_mm + "mm away!");
  /*
  if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
    System.out.println("The target is " + measurement.distance_mm + "mm away!");
  } else {
    System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");    
    // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.    
  }
    */
  return measurement.distance_mm;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
