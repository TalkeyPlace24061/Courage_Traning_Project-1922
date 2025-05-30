// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorDecector extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorsensor = new ColorSensorV3(i2cPort);
  /** Creates a new ColorDecector. */
  public ColorDecector() {}

  public void Detected(){
    Color detectedcolor = m_colorsensor.getColor();
    double IR = m_colorsensor.getIR();
    SmartDashboard.putNumber("Red", detectedcolor.red);
    SmartDashboard.putNumber("Blue", detectedcolor.blue);
    SmartDashboard.putNumber("Green", detectedcolor.green);
    SmartDashboard.putNumber("IR", IR);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
