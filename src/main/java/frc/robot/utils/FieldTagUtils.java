package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;

public class FieldTagUtils {
  private static final Alert tagLoadFailureAlert =
      new Alert("Failed to load custom tag map", AlertType.kWarning);

  private static AprilTagFieldLayout fieldTagLayout;

  /**
   * Loads the AprilTag field layout from the deploy directory
   *
   * @return The loaded AprilTagFieldLayout
   */
  public static AprilTagFieldLayout getFieldTagLayout() {
    if (fieldTagLayout == null) {
      try {
        fieldTagLayout =
            new AprilTagFieldLayout(
                Filesystem.getDeployDirectory()
                    .toPath()
                    .resolve("vision" + File.separator + "2025-reefscape.json"));
        System.out.println("Successfully loaded tag map");
      } catch (Exception e) {
        System.err.println("Failed to load custom tag map: " + e.getMessage());
        tagLoadFailureAlert.set(true);
        // Fall back to default field layout
        fieldTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
      }
    }
    return fieldTagLayout;
  }
}
