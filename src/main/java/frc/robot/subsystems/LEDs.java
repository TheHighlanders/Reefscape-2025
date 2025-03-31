// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.Frequency;
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
  Trigger isAligning;

  /** Creates a new LEDs. */
  private static final int kPort = 9;

  public static final double kledBrightness = 100;

  private static final int kLength = 120;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  public static final LEDPattern alignOk = LEDPattern.solid(Color.kGreen);
  LEDPattern allianceLED = LEDPattern.solid(Color.kRed);
  LEDPattern allianceColor;

  // LEDPattern pattern = blink.blink(Seconds.of(.1));
  public LEDs(Trigger canAlign, BooleanSupplier isAligning) {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      allianceLED = LEDPattern.solid(Color.kBlue);
    }

    this.canAlign = canAlign;
    this.isAligning = new Trigger(isAligning);

    this.canAlign
        .and(this.isAligning.negate())
        .onTrue(runPatternCommand(alignOk).withName("Alignment OK Pattern"))
        .onFalse(breathingPattern(getAllianceLed(), 1d).withName("Alignment NOT OK Pattern"));

    this.isAligning.onTrue(runPatternCommand(LEDPattern.solid(Color.kYellow)));
    this.isAligning.onFalse(breathingPattern(getAllianceLed(), 1d)); // one hundred yellow

    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();
    m_led.setColorOrder(ColorOrder.kRGB);

    runPatternCommand(allianceColor);

    this.setName("LEDs");
  }

  @Override
  public void periodic() {
    m_led.setData(m_buffer);
  }

  public void disabledInit() {
    runPattern(LEDPattern.solid(Color.kBlack));
  }

  public void disabledExit() {
    breathingPattern(getAllianceLed(), 1d).schedule();
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
    runPattern(pattern, kledBrightness);
  }

  public void runPattern(LEDPattern pattern, double ledBrightness) {
    pattern.atBrightness(Percent.of(ledBrightness)).applyTo(m_buffer);
  }

  public LEDPattern getAllianceLed() {
    return allianceLED;
  }

  /**
   * Creates a blinking pattern with the specified color and timing
   *
   * @param color The color to blink
   * @param onTimeSeconds On time in seconds
   * @param offTimeSeconds Off time in seconds
   * @return Command to run the blinking pattern
   */
  public Command blinkPattern(Color color, double onTimeSeconds, double offTimeSeconds) {
    LEDPattern pattern =
        LEDPattern.solid(color).blink(Seconds.of(onTimeSeconds), Seconds.of(offTimeSeconds));
    return runPatternCommand(pattern);
  }

  public Command blinkPattern(LEDPattern initPattern, double onTimeSeconds, double offTimeSeconds) {
    LEDPattern pattern = initPattern.blink(Seconds.of(onTimeSeconds), Seconds.of(offTimeSeconds));
    return runPatternCommand(pattern);
  }

  /**
   * Creates a breathing pattern with the specified color
   *
   * @param color The color to breathe
   * @param periodSeconds Breathing period in seconds
   * @return Command to run the breathing pattern
   */
  public Command breathingPattern(Color color, double periodSeconds) {
    LEDPattern pattern = LEDPattern.solid(color).breathe(Seconds.of(periodSeconds));
    return runPatternCommand(pattern);
  }

  public Command breathingPattern(LEDPattern initPattern, double periodSeconds) {
    LEDPattern pattern = initPattern.breathe(Seconds.of(periodSeconds));
    return runPatternCommand(pattern);
  }

  /**
   * Creates a pattern that synchronizes with a boolean signal
   *
   * @param color The color to display when signal is true
   * @param signal The boolean signal to synchronize with
   * @return Command to run the synchronized pattern
   */
  public Command synchronizedPattern(Color color, BooleanSupplier signal) {
    LEDPattern pattern = LEDPattern.solid(color).synchronizedBlink(signal);
    return runPatternCommand(pattern);
  }

  /**
   * Creates a scrolling pattern with the specified color
   *
   * @param color The color to scroll
   * @param scrollFrequency How fast to scroll (cycles per second)
   * @return Command to run the scrolling pattern
   */
  public Command scrollingPattern(Color color, double scrollFrequency) {
    LEDPattern pattern =
        LEDPattern.solid(color)
            .scrollAtRelativeSpeed(Frequency.ofBaseUnits(scrollFrequency, Hertz.getBaseUnit()));
    return runPatternCommand(pattern);
  }
}
