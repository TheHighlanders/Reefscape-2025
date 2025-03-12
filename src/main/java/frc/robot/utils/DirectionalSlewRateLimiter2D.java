package frc.robot.utils;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A slew rate limiter that considers 2D motion and applies different rate limits based on the
 * direction of movement. Particularly useful for swerve drive systems.
 */
public class DirectionalSlewRateLimiter2D {
  // Rate limits for different directions
  private final double[] m_rateLimits; // [forward, backward, left, right]

  // Previous values
  private Translation2d m_prevValue;
  private double m_prevTime;

  /**
   * Creates a new DirectionalSlewRateLimiter2D with specified rate limits for each direction.
   *
   * @param rateLimits Array of rate limits [forward, backward, left, right]
   * @param initialValue Initial translation value
   */
  public DirectionalSlewRateLimiter2D(double[] rateLimits, Translation2d initialValue) {
    if (rateLimits.length != 4) {
      throw new IllegalArgumentException(
          "Rate limits array must have exactly 4 elements [forward, backward, left, right]");
    }

    m_rateLimits = new double[4];
    for (int i = 0; i < 4; i++) {
      m_rateLimits[i] = Math.abs(rateLimits[i]);
    }

    m_prevValue = initialValue;
    m_prevTime = MathSharedStore.getTimestamp();
  }

  public DirectionalSlewRateLimiter2D(double[] rateLimits) {
    this(rateLimits, new Translation2d());
  }

  /**
   * Creates a DirectionalSlewRateLimiter2D with specified rate limits.
   *
   * @param forwardRateLimit Rate limit for forward motion (+X)
   * @param backwardRateLimit Rate limit for backward motion (-X)
   * @param leftRateLimit Rate limit for left motion (+Y)
   * @param rightRateLimit Rate limit for right motion (-Y)
   */
  public DirectionalSlewRateLimiter2D(
      double forwardRateLimit,
      double backwardRateLimit,
      double leftRateLimit,
      double rightRateLimit) {
    this(
        new double[] {
          Math.abs(forwardRateLimit),
          Math.abs(backwardRateLimit),
          Math.abs(leftRateLimit),
          Math.abs(rightRateLimit)
        },
        new Translation2d());
  }

  /**
   * Creates a DirectionalSlewRateLimiter2D with the same rate limit for all directions.
   *
   * @param rateLimit The rate limit to use for all directions
   */
  public DirectionalSlewRateLimiter2D(double rateLimit) {
    this(rateLimit, rateLimit, rateLimit, rateLimit);
  }

  /**
   * Filters the input to limit its slew rate based on direction of movement.
   *
   * @param input The desired Translation2d value
   * @return The filtered Translation2d value after applying rate limits
   */
  public Translation2d calculate(Translation2d input) {
    double currentTime = MathSharedStore.getTimestamp();
    double elapsedTime = currentTime - m_prevTime;

    // Calculate desired change
    Translation2d delta = input.minus(m_prevValue);

    // If no change or no elapsed time, return current values
    if (delta.getNorm() == 0 || elapsedTime == 0) {
      return m_prevValue;
    }

    // Determine which directional limit to use
    double rateLimit;
    double dx = delta.getX();
    double dy = delta.getY();

    // Determine the primary direction based on the largest component
    if (Math.abs(dx) >= Math.abs(dy)) {
      // Movement is primarily in X axis
      rateLimit = dx > 0 ? m_rateLimits[0] : m_rateLimits[1]; // forward or backward
    } else {
      // Movement is primarily in Y axis
      rateLimit = dy > 0 ? m_rateLimits[2] : m_rateLimits[3]; // left or right
    }

    // Calculate magnitude of desired change
    double deltaMagnitude = delta.getNorm();

    // Calculate maximum allowed change based on rate limit and elapsed time
    double maxChange = rateLimit * elapsedTime;

    // If desired change exceeds maximum allowed change, scale the delta
    if (deltaMagnitude > maxChange) {
      delta = delta.times(maxChange / deltaMagnitude);
    }

    // Apply limited change
    Translation2d newValue = m_prevValue.plus(delta);

    // Update previous values
    m_prevValue = newValue;
    m_prevTime = currentTime;

    return newValue;
  }

  /**
   * Convenience method that takes x,y components and returns a Translation2d.
   *
   * @param x The input x value
   * @param y The input y value
   * @return Translation2d with the filtered x and y values
   */
  public Translation2d calculate(double x, double y) {
    return calculate(new Translation2d(x, y));
  }

  /**
   * Convenience method that takes x,y components and returns filtered components as a double array.
   *
   * @param x The input x value
   * @param y The input y value
   * @return Double array [x, y] with the filtered values
   */
  public double[] calculateArray(double x, double y) {
    Translation2d result = calculate(new Translation2d(x, y));
    return new double[] {result.getX(), result.getY()};
  }

  /**
   * Returns the most recent value calculated by the DirectionalSlewRateLimiter2D.
   *
   * @return The last calculated translation value
   */
  public Translation2d lastValue() {
    return m_prevValue;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The Translation2d value to reset to
   */
  public void reset(Translation2d value) {
    m_prevValue = value;
    m_prevTime = MathSharedStore.getTimestamp();
  }

  /**
   * Resets the slew rate limiter to the specified x,y values; ignores the rate limit when doing so.
   *
   * @param x The x value to reset to
   * @param y The y value to reset to
   */
  public void reset(double x, double y) {
    reset(new Translation2d(x, y));
  }
}
