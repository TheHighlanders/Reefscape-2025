// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDs extends SubsystemBase {
  private static final int kPort = 9;
  private static final int kLength = 120;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  public LEDS() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();

    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));


  // // all hues at maximum saturation and half brightness
  // private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

  // // Our LED strip has a density of 120 LEDs per meter
  // private static final Distance kLedSpacing = Meters.of(1 / 120.0);

  // // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
  // // of 1 meter per second.
  // private final LEDPattern m_scrollingRainbow =
  //     m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

  }


  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display
    m_led.setData(m_buffer);

    
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