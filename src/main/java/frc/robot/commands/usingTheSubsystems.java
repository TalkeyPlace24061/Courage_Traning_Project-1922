// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ServoMotor;
import frc.robot.subsystems.Elastic;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Neo;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class usingTheSubsystems extends Command {
  /** Creates a new SetAngle. */
  ServoMotor m_ServoMotor;
  LED m_LED;
  Neo m_Neo;
  public usingTheSubsystems(ServoMotor servoMotor, LED led, Neo neo) {
    addRequirements(servoMotor);
    // Use addRequirements() here to declare subsystem dependencies.
    m_ServoMotor = servoMotor;
    m_LED = led;
    m_Neo = neo;
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
      m_Neo.moveNeo(1);
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
