// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Align;

public class LEDs extends SubsystemBase {
  Vision vision;
  Swerve swerve;

  /** Creates a new LEDs. */
  private static final int kPort = 9;

  private static final int kLength = 120;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  public LEDs(Swerve swerve, Vision vision) {
    this.swerve = swerve;
    this.vision = vision;

    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();
    m_led.setColorOrder(ColorOrder.kRGB);
    
    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    

        if(DriverStation.getAlliance().isPresent()){
          if(DriverStation.getAlliance().get() == Alliance.Red){
            runPattern(LEDPattern.solid(Color.kRed)).withName("Red").ignoringDisable(true).schedule();
          }
          if(DriverStation.getAlliance().get() == Alliance.Blue){
            runPattern(LEDPattern.solid(Color.kBlue)).withName("Blue").ignoringDisable(true).schedule();
          }
        }else{
          runPattern(LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue)).withName("GradientRedBlue").ignoringDisable(true).schedule();
        }
    }
  

  
  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display
    m_led.setData(m_buffer);
    
    
    if (Align.canAlign(swerve, vision)){
      // LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
      // runPattern(gradient);
      LEDPattern blink = LEDPattern.solid(Color.kWhite);
      LEDPattern pattern = blink.blink(Seconds.of(.5));
      runPattern(pattern);
    }
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer));
  }
}
