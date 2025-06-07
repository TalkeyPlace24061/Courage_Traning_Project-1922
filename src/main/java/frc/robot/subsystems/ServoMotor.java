// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ServoMotor extends SubsystemBase {
  Servo exampleServo = new Servo(1);
 double m_angle;
  /** Creates a new Servo. */
  public ServoMotor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    Elastic.Notification notification = new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, "Error Notification", "This is an example error notification.");
  Elastic.sendNotification(notification);  
 m_angle = SmartDashboard.getNumber("Number",1);
 SmartDashboard.putNumber("test", m_angle);

}


  public void setAngle(){
    exampleServo.set(m_angle);
  }
}
