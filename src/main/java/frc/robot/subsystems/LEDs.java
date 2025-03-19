// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

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
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class LEDs extends SubsystemBase {

  /** Enum defining different LED states with associated patterns */
  public enum LEDState {
    OFF(LEDPattern.solid(Color.kBlack)),
    // Alliance colors
    ALLIANCE(getAlliancePattern()),
    ALLIANCE_GRADIENT(
        LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue)),
    // Status indicators
    ALIGNMENT_OK(LEDPattern.solid(Color.kGreen)),
    ALIGNMENT_CLOSE(LEDPattern.solid(Color.kYellow)),
    ALIGNMENT_BAD(LEDPattern.solid(Color.kRed).blink(Seconds.of(0.25))),
    // Game piece states
    INTAKE_ACTIVE(LEDPattern.solid(Color.kOrange).breathe(Seconds.of(1.0))),
    // Shooting states
    SHOOTING_READY(LEDPattern.solid(Color.kPurple)),
    SHOOTING_ACTIVE(LEDPattern.solid(Color.kPurple).blink(Seconds.of(0.1))),
    // Special patterns
    RAINBOW(LEDPattern.rainbow(255, 255)),
    BREATHING_RAINBOW(LEDPattern.rainbow(255, 255).breathe(Seconds.of(2.0))),
    ERROR(LEDPattern.solid(Color.kRed).blink(Seconds.of(0.2))),
    CLIMBING(createClimbingPattern()),
    ENDGAME(createEndgamePattern());

    private final LEDPattern pattern;

    LEDState(LEDPattern pattern) {
      this.pattern = pattern;
    }

    public LEDPattern getPattern() {
      return pattern;
    }

    public LEDPattern alignPattern(BooleanSupplier canAlign) {
      return ALIGNMENT_OK.getPattern().synchronizedBlink(canAlign).overlayOn(ALLIANCE.getPattern());
    }

    // Helper method to create a climbing pattern with progress indication
    private static LEDPattern createClimbingPattern() {
      return LEDPattern.gradient(
          LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kYellow, Color.kRed);
    }

    // Helper method to create an endgame warning pattern
    private static LEDPattern createEndgamePattern() {
      return LEDPattern.solid(Color.kYellow)
          .blink(Seconds.of(0.5), Seconds.of(0.5))
          .overlayOn(LEDPattern.solid(Color.kRed));
    }

    private static LEDPattern getAlliancePattern() {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get().equals(Alliance.Red)
            ? LEDPattern.solid(Color.kRed)
            : LEDPattern.solid(Color.kBlue);
      } else {
        return LEDPattern.solid(Color.kWhite);
      }
    }
  }

  private static final int kPort = 9;
  private static final int kLength = 120;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;
  private LEDState currentState;

  private Trigger canAlign;
  private BooleanSupplier isClimbing = () -> false;
  private DoubleSupplier climbProgress = () -> 0.0;

  /** Creates a new LEDs. */
  /** Creates a new LEDs. */
  public LEDs(Trigger canAlign) {
    this.canAlign = canAlign;

    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.setColorOrder(ColorOrder.kRGB);
    m_led.start();

    currentState = LEDState.ALLIANCE;

    LEDPattern alignmentPattern = LEDState.ALLIANCE.alignPattern(canAlign);
    alignmentPattern.applyTo(m_buffer);

    this.setName("LEDs");
  }

  @Override
  public void periodic() {
    m_led.setData(m_buffer);
  }

  /**
   * Sets the LED state and applies the corresponding pattern
   *
   * @param state The LED state to set
   * @return Command that sets the LED state
   */
  public Command setState(LEDState state) {
    return runOnce(
            () -> {
              currentState = state;
              state.getPattern().applyTo(m_buffer);
            })
        .withName("Set LED State: " + state.name());
  }

  /**
   * Sets the LEDs to the appropriate alliance color
   *
   * @return Command that sets alliance-based LED state
   */
  public Command setToAllianceState() {
    return runOnce(
            () -> {
              setState(LEDState.ALLIANCE);
            })
        .withName("Set Alliance LED State");
  }

  /**
   * Gets the current LED state
   *
   * @return The current LED state
   */
  public LEDState getCurrentState() {
    return currentState;
  }

  /**
   * Creates a command that runs a custom pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer)).withName("Run Custom LED Pattern");
  }

  /**
   * Sets up elevator mode with progress indication
   *
   * @param isElevatorActive Boolean supplier indicating if elevator is active
   * @param progressSupplier Double supplier providing elevator height progress (0.0 to 1.0)
   * @return Command to enable elevator progress indication
   */
  public Command setElevatorMode(
      BooleanSupplier isElevatorActive, DoubleSupplier progressSupplier) {
    return runOnce(
            () -> {
              if (isElevatorActive.getAsBoolean()) {
                // Create a base pattern - could be a gradient or solid color
                LEDPattern basePattern =
                    LEDPattern.gradient(
                        LEDPattern.GradientType.kContinuous,
                        Color.kBlue, // Bottom color
                        Color.kGreen // Top color
                        );

                // Create a progress mask based on elevator height
                LEDPattern progressMask = LEDPattern.progressMaskLayer(progressSupplier);

                // Apply the mask to the base pattern
                LEDPattern resultPattern = progressMask.mask(basePattern);

                // Apply to LEDs
                resultPattern.applyTo(m_buffer);
              } else {
                // Return to default state when elevator is inactive
                setToAllianceState();
              }
            })
        .withName("Set Elevator Progress Mode");
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
    return runPattern(pattern);
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
    return runPattern(pattern);
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
    return runPattern(pattern);
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
    return runPattern(pattern);
  }

  public Command refreshPattern() {
    return runOnce(
            () -> {
              if (currentState == LEDState.ALLIANCE) {
                LEDPattern alliancePattern = LEDState.getAlliancePattern();
                alliancePattern.applyTo(m_buffer);
              }
            })
        .withName("Refresh LED Pattern");
  }
}
