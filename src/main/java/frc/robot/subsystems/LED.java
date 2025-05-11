// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import java.util.RandomAccess;

import edu.wpi.first.hal.LEDJNI;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.usingTheSubsystems;
import frc.robot.subsystems.ServoMotor;

public class LED extends SubsystemBase {
  private  AddressableLED  m_led;
  private  AddressableLEDBuffer m_ledBuffer;

  /** Creates a new LED. */
  public LED(usingTheSubsystems usingSubsystems) {
   ServoMotor m_servo;

  m_led = new AddressableLED(0);
  m_ledBuffer = new AddressableLEDBuffer(3);
  m_led.setLength(m_ledBuffer.getLength());
  m_led.start(); //Do not comment these lines out they create the LEDs 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

   /*LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
   LEDPattern pattern = base.blink(Seconds.of(.5),Seconds.of(.5));
   pattern.applyTo(m_ledBuffer);
   m_led.setData(m_ledBuffer);*/

   /*LEDPattern red = LEDPattern.solid(Color.kGreen);
   red.applyTo(m_ledBuffer);
   m_led.setData(m_ledBuffer);*/

   LEDPattern.solid(new Color(90,89,70)).applyTo(m_ledBuffer);
  }

}
