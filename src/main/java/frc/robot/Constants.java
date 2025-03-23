// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  // Change to static

  private static final AprilTagFields currentField = AprilTagFields.k2025ReefscapeAndyMark;
  protected static AprilTagFieldLayout fieldTags;

  // FL, FR, BL, BR
  public static final int[] driveMotorIDs = {3, 11, 21, 31};
  public static final int[] angleMotorIDs = {2, 12, 22, 32};
  public static final int[] canCoderIDs = {3, 13, 23, 33};

  public static final double maxSpeed = Units.feetToMeters(15);

  public static final boolean sim = RobotBase.isSimulation();

  public static final boolean devMode = false;

  // Add alerts for tag loading status
  private static final Alert tagLoadFailureAlert =
      new Alert("Failed to load custom tag map", AlertType.kWarning);

  // Static initialization block - runs when the class is loaded
  static {
    // Initialize fieldTags
    // JSON files here:
    // https://github.com/wpilibsuite/allwpilib/blob/71b6e8ec583ab23c5de014ff899ec956f04096fd/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2025-reefscape-andymark.json#L4
    try {
      fieldTags =
          new AprilTagFieldLayout(
              Filesystem.getDeployDirectory()
                  .toPath()
                  .resolve("vision" + File.separator + currentField.m_resourceFile));
      System.out.println("Successfully loaded tag map");
    } catch (Exception e) {
      System.err.println("Failed to load custom tag map: " + e.getMessage());
      tagLoadFailureAlert.set(true);
      // Fall back to default field layout
      fieldTags = AprilTagFieldLayout.loadField(currentField);
    }
  }

  /**
   * Gets the field tag layout
   *
   * @return The AprilTagFieldLayout for the field
   */
  public static AprilTagFieldLayout getFieldTagLayout() {
    return fieldTags;
  }
}
