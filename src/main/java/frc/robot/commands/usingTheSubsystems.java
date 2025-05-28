// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ServoMotor;
import frc.robot.subsystems.Elastic;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Neo;
import frc.robot.subsystems.TimeOfFlight;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class usingTheSubsystems extends Command {

  ServoMotor m_ServoMotor;
  LED m_LED;
  Neo m_Neo;
  TimeOfFlight m_TOF;
    /** Creates a new SetAngle.

   */
  public usingTheSubsystems(ServoMotor servoMotor, LED led, Neo neo, TimeOfFlight TOF) {
    addRequirements(servoMotor);
    // Use addRequirements() here to declare subsystem dependencies.
    m_ServoMotor = servoMotor;
    m_LED = led;
    m_Neo = neo;
    m_TOF = TOF;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Number",10);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_ServoMotor.setAngle();

      var distance =  m_TOF.Distance();
      double speed = distance/800;
      
      if (distance < 100) {
        speed = 0.2;
        m_LED.Colors(Color.kBlue);
      } else if (distance < 200) {
        speed = 0.4;
        m_LED.Colors(Color.kPurple);
      } else if (distance < 300) {
        speed = 0.6;
        m_LED.Colors(Color.kBlue);
      } else if (distance < 400) {
        speed = 0.8;
        m_LED.Colors(Color.kRed);
      } else {
        speed = 1;
        m_LED.Colors(Color.kWhite);
      }
      
      
      System.out.println("The speed is " + speed);
      
      m_Neo.moveNeo(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
