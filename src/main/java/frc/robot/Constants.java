package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  // FL, FR, BL, BR
  public static final int[] driveMotorIDs = {1, 11, 21, 31};
  public static final int[] angleMotorIDs = {2, 12, 22, 32};
  public static final double[] absoluteOffsets = {
    347.1482491493225, 307, 237.8471857309, 256.65536928176877
  };

  public static final double maxSpeed = Units.feetToMeters(13);

  public static final boolean sim = RobotBase.isSimulation();
}
