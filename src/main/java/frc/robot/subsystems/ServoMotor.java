// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ServoMotor extends SubsystemBase {
  Servo exampleServo = new Servo(1);
  /** Creates a new Servo. */
  public ServoMotor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAngle() throws InterruptedException{
    exampleServo.setAngle(90);
    //wait(2000);
    exampleServo.setAngle(45);
  }
}
