// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import choreo.trajectory.SwerveSample;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

class SwerveConstants {

  public static final double maxRotSpeed = Units.degreesToRadians(180);
  // Implicit /sec

  static double width = Units.inchesToMeters(20.5);
  static double length = width;

  static double maxSpeed = Constants.maxSpeed;
  // Implict /sec

  static double accelLim = 1.5;

  static double translateP = 0;
  static double translateI = 0;
  static double translateD = 0;

  static double rotateP = 0;
  static double rotateI = 0;
  static double rotateD = 0;
}

public class Swerve extends SubsystemBase {

  enum SwerveState {
    FAST,
    SLOW
  }

  private static final double SLOW_MODE_MULTIPLIER = 0.3;

  Module[] modules = new Module[4];
  AHRS gyro;
  SwerveDrivePoseEstimator poseEst;
  SwerveDriveKinematics kinematics;
  Pose2d startPose = new Pose2d(0, 0, new Rotation2d());

  StructArrayPublisher<SwerveModuleState> statePublisher;
  StructArrayPublisher<SwerveModuleState> setpointPublisher;
  DoublePublisher voltagePublisher;
  StructPublisher<Pose2d> posePublisher;

  SlewRateLimiter xLim = new SlewRateLimiter(SwerveConstants.accelLim);
  SlewRateLimiter yLim = new SlewRateLimiter(SwerveConstants.accelLim);

  private final PIDController xController =
      new PIDController(
          SwerveConstants.translateP, SwerveConstants.translateI, SwerveConstants.translateD);
  private final PIDController yController =
      new PIDController(
          SwerveConstants.translateP, SwerveConstants.translateI, SwerveConstants.translateD);
  private final PIDController headingController =
      new PIDController(SwerveConstants.rotateP, SwerveConstants.rotateI, SwerveConstants.rotateD);

  GenericEntry drivePEntry;
  GenericEntry driveIEntry;
  GenericEntry driveDEntry;

  GenericEntry driveSEntry;
  GenericEntry driveVEntry;
  GenericEntry driveAEntry;

  GenericEntry anglePEntry;
  GenericEntry angleIEntry;
  GenericEntry angleDEntry;

  private final SysIdRoutine sysId;

  SwerveState current = SwerveState.FAST;

  /** Creates a new Swerve. */
  public Swerve() {
    for (int i = 0; i < modules.length; i++) {
      modules[i] = new Module(i);
    }

    // Using +X as forward, and +Y as left, as per
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    // FL, FR, BL, BR
    double y = SwerveConstants.width / 2.0d;
    double x = SwerveConstants.length / 2.0d;

    kinematics =
        new SwerveDriveKinematics(
            new Translation2d(x, y),
            new Translation2d(x, -y),
            new Translation2d(-x, y),
            new Translation2d(-x, -y));

    // Default Port is MXP
    gyro = new AHRS(NavXComType.kMXP_SPI);

    poseEst =
        new SwerveDrivePoseEstimator(
            kinematics, startPose.getRotation(), getModulePostions(), startPose);

    statePublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("/Swerve/States", SwerveModuleState.struct)
            .publish();
    setpointPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("/Swerve/Setpoints", SwerveModuleState.struct)
            .publish();

    posePublisher =
        NetworkTableInstance.getDefault().getStructTopic("/Swerve/Poses", Pose2d.struct).publish();

    voltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/Voltage").publish();

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volt.of(4),
                Seconds.of(4),
                state -> {
                  SmartDashboard.putString("Drive/SysIdState", state.toString());
                }),
            new SysIdRoutine.Mechanism(
                voltage -> {
                  driveVoltage(voltage);
                },
                log -> {
                  log.motor("Front-Left")
                      .voltage(
                          Volts.of(
                              modules[0].getDriveVolts().in(Volts)
                                  * RobotController.getBatteryVoltage()))
                      .linearPosition(Meters.of(modules[0].getDrivePosition()))
                      .linearVelocity(MetersPerSecond.of(modules[0].getDriveVelocity()));
                  log.motor("Front-Right")
                      .voltage(
                          Volts.of(
                              modules[1].getDriveVolts().in(Volts)
                                  * RobotController.getBatteryVoltage()))
                      .linearPosition(Meters.of(modules[1].getDrivePosition()))
                      .linearVelocity(MetersPerSecond.of(modules[1].getDriveVelocity()));
                },
                this));

    smartDashboardGUI();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    poseEst.updateWithTime(
        RobotController.getFPGATime() * Math.pow(10, 6), getGyroAngle(), getModulePostions());

    sendNT();
  }

  @Override
  public void simulationPeriodic() {
    for (Module m : modules) {
      m.updateSimMotors();
      SmartDashboard.putNumber(
          "Drive Voltage Module" + m.getModuleNumber(), m.getAppliedVoltageDrive());
    }
  }

  public Pose2d getPose() {
    return poseEst.getEstimatedPosition();
  }

  public SwerveModulePosition[] getModulePostions() {
    List<SwerveModulePosition> out = new ArrayList<SwerveModulePosition>();
    for (Module mod : modules) {
      out.add(mod.getPosition());
    }
    return out.toArray(new SwerveModulePosition[0]);
  }

  public SwerveModuleState[] getModuleStates() {
    List<SwerveModuleState> out = new ArrayList<SwerveModuleState>();
    for (Module mod : modules) {
      out.add(mod.getState());
    }
    return out.toArray(new SwerveModuleState[0]);
  }

  public Command readAngleEncoders() {
    return new InstantCommand(
            () -> {
              for (Module m : modules) {
                SmartDashboard.putNumber(
                    "Relative" + m.moduleNumber, m.getAnglePosition().getDegrees());
                SmartDashboard.putNumber(
                    "Absolute" + m.moduleNumber, m.getAbsolutePosition().getDegrees());
              }
            },
            this)
        .ignoringDisable(true);
  }

  public void resetEncoders() {
    for (Module m : modules) {
      m.setIntegratedAngleToAbsolute();
    }
  }

  public SwerveModuleState[] getModuleSetpoints() {
    List<SwerveModuleState> out = new ArrayList<SwerveModuleState>();
    for (Module mod : modules) {
      out.add(mod.getSetpoint());
    }
    return out.toArray(new SwerveModuleState[0]);
  }

  public Rotation2d getGyroAngle() {
    return gyro.getRotation2d();
  }

  public void resetOdometry(Pose2d pose) {
    poseEst.resetPosition(pose.getRotation(), getModulePostions(), pose);
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public Command sysId() {
    return Commands.sequence(
        sysIdDynamic(Direction.kForward),
        Commands.waitSeconds(1),
        sysIdDynamic(Direction.kReverse),
        Commands.waitSeconds(1),
        sysIdQuasistatic(Direction.kForward),
        Commands.waitSeconds(1),
        sysIdQuasistatic(Direction.kReverse));
  }

  /*
   * Alliance Relative Coords
   * +X is to the driver behind the glass's right
   * +Y is away from the driver
   */

  /**
   * @param x Supplier for desired Alliance Relative X translation
   * @param y Supplier for desired Alliance Relative Y translation
   * @return Drive Command
   */
  public Command driveCMD(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {

    return new RunCommand(
            () -> {
              drive(x.getAsDouble(), y.getAsDouble(), omega.getAsDouble());
            },
            this)
        .withName("Swerve Drive Command");
  }

  /**
   * Method to drive the robot
   *
   * @param x Alliance Relative X Speed, as defined above (m/s)
   * @param y Alliance Relative Y Speed, as defined above (m/s)
   * @param omega Rotational Speed (rad/s)
   */
  public void drive(double x, double y, double omega) {
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html

    if (current == SwerveState.SLOW) {
      x *= SLOW_MODE_MULTIPLIER;
      y *= SLOW_MODE_MULTIPLIER;
    }

    ChassisSpeeds chassisSpeeds =
        fromAllianceRelativeSpeeds(
            xLim.calculate(x),
            yLim.calculate(y),
            omega); // Takes in Alliance Relative, returns Field Relative

    chassisSpeeds.vxMetersPerSecond *= SwerveConstants.maxSpeed;
    chassisSpeeds.vyMetersPerSecond *= SwerveConstants.maxSpeed;
    chassisSpeeds.omegaRadiansPerSecond *= SwerveConstants.maxRotSpeed;

    driveChassisSpeedsRobotRelative(chassisSpeeds);
  }

  public void driveChassisSpeedsRobotRelative(ChassisSpeeds chassisSpeeds) {
    // https://github.com/wpilibsuite/allwpilib/issues/7332

    // Convert to States and desat
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveConstants.maxSpeed);

    // Convert to ChassisSpeeds and discretize
    chassisSpeeds = kinematics.toChassisSpeeds(targetStates);
    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

    // Convert back to States, and desat, again
    targetStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveConstants.maxSpeed);

    for (int i = 0; i < modules.length; i++) {
      targetStates[i].optimize(getModulePostions()[i].angle);
      modules[i].setModuleState(targetStates[i], false);
    }
  }

  public void sendNT() {
    statePublisher.set(getModuleStates());
    setpointPublisher.set(getModuleSetpoints());
    posePublisher.set(poseEst.getEstimatedPosition());
    SmartDashboard.putNumber(
        "Hypot",
        Math.pow(poseEst.getEstimatedPosition().getX(), 2)
            + Math.pow(poseEst.getEstimatedPosition().getY(), 2));

    voltagePublisher.set(RoboRioSim.getVInVoltage());
  }

  /**
   * Create Field Relative IN CHASSIS SPEEDS COORD SYSTEM Chassis Speeds from Alliance Relative
   * desired speeds
   *
   * @param arx Alliance relative desired X speed
   * @param ary Alliance relative desired Y speed
   * @param rot Rotation speed, direction does not differ between alliances
   * @return Field Relative Chassis Speeds
   */
  public ChassisSpeeds fromAllianceRelativeSpeeds(double arx, double ary, double rot) {
    boolean isRedAlliance = true;
    ChassisSpeeds fr; // Field Relative

    // Rotation2d deg180 = Rotation2d.fromDegrees(180);

    Translation2d allianceRelativeSpeeds = new Translation2d(arx, ary);

    Translation2d fieldRelativeSpeeds;

    if (DriverStation.getAlliance().isPresent()) {
      isRedAlliance = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    } else {
      DriverStation.reportError("No Alliance Present, Defaulting to RED", false);
    }

    if (isRedAlliance) {
    } else { // RED ALLIANCE CASE //BLUE ALLIANCE CASE
      allianceRelativeSpeeds =
          new Translation2d(-allianceRelativeSpeeds.getX(), -allianceRelativeSpeeds.getY());
    }

    // Convert Blue Alliance Relative to Field Relative
    // fieldRelativeSpeeds =
    // allianceRelativeSpeeds.rotateBy(Rotation2d.fromDegrees(90));
    fieldRelativeSpeeds =
        new Translation2d(-allianceRelativeSpeeds.getY(), -allianceRelativeSpeeds.getX());

    fr = new ChassisSpeeds(fieldRelativeSpeeds.getX(), fieldRelativeSpeeds.getY(), rot);
    fr = ChassisSpeeds.fromFieldRelativeSpeeds(fr, getPose().getRotation());

    return fr;
  }

  public Command slowMode() {
    return new RunCommand(() -> current = SwerveState.SLOW, this)
        .finallyDo(() -> current = SwerveState.FAST);
  }

  public void driveVoltage(Measure<VoltageUnit> voltage) {
    for (Module m : modules) {
      m.driveVolts(voltage);
    }
  }

  public void followTraj(SwerveSample sample) {
    // Get the current pose of the robot
    Pose2d pose = getPose();

    // Generate the next speeds for the robot
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            -(sample.vx + xController.calculate(pose.getX(), sample.x)),
            -(sample.vy + yController.calculate(pose.getY(), sample.y)),
            -(sample.omega
                + headingController.calculate(pose.getRotation().getRadians(), sample.heading)));

    SmartDashboard.putNumber("Trajectory/XError", xController.getError());
    SmartDashboard.putNumber("Trajectory/YError", yController.getError());
    SmartDashboard.putNumber("Trajectory/HeadingError", headingController.getError());

    // SmartDashboard.putNumber("Trajectory/XOutput",
    // xController.calculate(pose.getX(), sample.x));
    // SmartDashboard.putNumber("Trajectory/YOutput",
    // yController.calculate(pose.getX(), sample.x));
    // SmartDashboard.putNumber("Trajectory/HeadingOutput",
    // headingController.calculate(pose.getRotation().getRadians(),
    // sample.heading));
    sendDiagnositics();
    // Apply the generated speeds
    driveChassisSpeedsRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation()));
  }

  public void sendDiagnositics() {
    for (Module m : modules) {
      SmartDashboard.putNumber(
          "ModuleDebug/Module" + m.getModuleNumber() + "FFoutput", m.getFFDriveOutput());
      SmartDashboard.putNumber(
          "ModuleDebug/Module" + m.getModuleNumber() + "MotorOutput", m.getAppliedOutputDrive());
    }
  }

  public Command resetGyro() {
    return new InstantCommand(
            () -> {
              poseEst.resetPosition(
                  getGyroAngle(),
                  getModulePostions(),
                  new Pose2d(
                      poseEst.getEstimatedPosition().getX(),
                      poseEst.getEstimatedPosition().getY(),
                      new Rotation2d()));
            })
        .ignoringDisable(true);
  }

  public Command resetOdometry() {
    return new InstantCommand(
        () -> {
          poseEst.resetTranslation(new Translation2d());
          resetGyro();
        });
  }

  public void setNewControlConstants(double[] drive, double[] angle) {
    for (Module m : modules) {
      m.setNewControlConstants(drive, angle);
    }
  }

  public void smartDashboardGUI() {
    ShuffleboardTab tab = Shuffleboard.getTab("Modules");
    Module m = modules[0];

    ShuffleboardLayout module =
        tab.getLayout("Tuning", BuiltInLayouts.kGrid)
            .withSize(2, 3)
            .withProperties(
                Map.of(
                    "Label position",
                    "LEFT",
                    "Number of columns",
                    1,
                    "Number of rows",
                    9,
                    "Retained",
                    true));
    anglePEntry =
        module
            .add("AngleP", m.getAngleP())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .withProperties(Map.of("retained", true))
            .getEntry();
    angleIEntry =
        module
            .add("AngleI", m.getAngleI())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 1)
            .withProperties(Map.of("retained", true))
            .getEntry();
    angleDEntry =
        module
            .add("AngleD", m.getAngleD())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 2)
            .withProperties(Map.of("retained", true))
            .getEntry();

    anglePEntry = configureWidget(module.add("AngleP", m.getAngleP()), 0);
    angleIEntry = configureWidget(module.add("AngleI", m.getAngleI()), 1);

    angleDEntry = configureWidget(module.add("AngleD", m.getAngleD()), 2);

    drivePEntry = configureWidget(module.add("DriveP", m.getDriveP()), 3);

    driveIEntry = configureWidget(module.add("DriveI", m.getDriveI()), 4);

    driveDEntry = configureWidget(module.add("DriveD", m.getDriveD()), 5);

    driveSEntry = configureWidget(module.add("DriveS", m.getDriveS()), 6);

    driveVEntry = configureWidget(module.add("DriveV", m.getDriveV()), 7);

    driveAEntry = configureWidget(module.add("DriveA", m.getDriveA()), 8);
  }

  public void updateDashboardGUI() {
    Module m = modules[0];

    anglePEntry.setDouble(m.getAngleP());
    angleIEntry.setDouble(m.getAngleI());
    angleDEntry.setDouble(m.getAngleD());

    drivePEntry.setDouble(m.getDriveP());
    driveIEntry.setDouble(m.getDriveI());
    driveDEntry.setDouble(m.getDriveD());

    driveSEntry.setDouble(m.getDriveS());
    driveVEntry.setDouble(m.getDriveV());
    driveAEntry.setDouble(m.getDriveA());
  }

  public void updateControlConstants() {
    double[] drive = {
      drivePEntry.getDouble(moduleConstants.driveP),
      driveIEntry.getDouble(moduleConstants.driveI),
      driveDEntry.getDouble(moduleConstants.driveD),
      driveSEntry.getDouble(moduleConstants.driveS),
      driveVEntry.getDouble(moduleConstants.driveV),
      driveAEntry.getDouble(moduleConstants.driveA),
    };

    double[] angle = {
      anglePEntry.getDouble(moduleConstants.angleP),
      angleIEntry.getDouble(moduleConstants.angleI),
      angleDEntry.getDouble(moduleConstants.angleD)
    };

    for (Module m : modules) {
      m.setNewControlConstants(drive, angle);
    }

    updateDashboardGUI();
  }

  public GenericEntry configureWidget(SimpleWidget widget, int row) {
    return widget
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, row)
        .withProperties(Map.of("retained", true))
        .getEntry();
  }
}
