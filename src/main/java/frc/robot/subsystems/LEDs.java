// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LEDs extends SubsystemBase {

  Trigger canAlign;

  /** Creates a new LEDs. */
  private static final int kPort = 9;

  public static final double ledBrightness = 100;

  private static final int kLength = 120;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  public static final LEDPattern alignOk =
      LEDPattern.solid(Color.kGreen).atBrightness(Percent.of(ledBrightness));
  LEDPattern allianceLED = LEDPattern.solid(Color.kRed).atBrightness(Percent.of(ledBrightness));
  LEDPattern allianceColor;

  // LEDPattern pattern = blink.blink(Seconds.of(.1));
  public LEDs(Trigger canAlign) {

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      allianceLED = LEDPattern.solid(Color.kBlue).atBrightness(Percent.of(ledBrightness));
    }

    this.canAlign = canAlign;

    canAlign
        .onTrue(runPatternCommand(alignOk).withName("Alignment OK Pattern"))
        .onFalse(runPatternCommand(allianceLED).withName("Alignment NOT OK Pattern"));

    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();
    m_led.setColorOrder(ColorOrder.kRGB);

    // Set the default command to turn the strip off, otherwise the last colors
    // written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    if (DriverStation.getAlliance().isPresent()) {
      // if (DriverStation.getAlliance().get() == Alliance.Red) {
      //   // setDefaultCommand(
      //   //     runPattern(LEDPattern.solid(Color.kRed))
      //   //         .withName("Red Alliance Pattern")
      //   //         .ignoringDisable(true));
      //   allianceLED
      // }

    }
    // else {
    //   setDefaultCommand(
    //       runPattern(
    //               LEDPattern.gradient(
    //                   LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue))
    //           .withName("Red-Blue Gradient Pattern")
    //           .ignoringDisable(true));
    // }

    runPatternCommand(allianceColor);

    this.setName("LEDs");
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to
    // display
    m_led.setData(m_buffer);

    // LEDPattern gradient =
    // LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed,
    // Color.kBlue);
    // runPattern(gradient);
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPatternCommand(LEDPattern pattern) {
    return runOnce(() -> runPattern(pattern)).withName("Run LED Pattern");
  }

  public void runPattern(LEDPattern pattern) {
    pattern.applyTo(m_buffer);
  }

  public LEDPattern getAllianceLed() {
    return allianceLED;
  }
}
