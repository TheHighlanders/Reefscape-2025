package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  // FL, FR, BL, BR
  public static final int[] driveMotorIDs = {3, 11, 21, 31};
  public static final int[] angleMotorIDs = {2, 12, 22, 32};
  public static final int[] canCoderIDs = {3, 13, 23, 33};

  public static final double maxSpeed = Units.feetToMeters(15);

  public static final boolean sim = RobotBase.isSimulation();

  public static final boolean devMode = false;
}
