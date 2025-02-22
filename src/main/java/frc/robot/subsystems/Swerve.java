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
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

final class SwerveConstants {

  public static final double maxRotSpeed = Units.degreesToRadians(270);
  // Implicit /sec

  static final double width = Units.inchesToMeters(20.5);
  static final double length = width;

  static final double maxSpeed = Constants.maxSpeed;
  // Implict /sec

  static final double accelLim = 3;

  static double translateP = 0.07500000298023224;
  static double translateI = 0;
  static double translateD = 0.05000000074505806;

  static double rotateP = 0;
  static double rotateI = 0;
  static double rotateD = 0;
}

public class Swerve extends SubsystemBase {

  enum SwerveState {
    NORMAL,
    LINEUP,
    SLOW
  }

  private static final double MAX_SLOW_MODE = 0.3;
  private static final double MICROS_SECONDS_CONVERSION = Math.pow(10, 6);

  private static final double MIN_HEIGHT_PERCENTAGE_TO_LIMIT_SPEED = 0.25;

  DoubleSupplier elevatorHeight;

  Module[] modules = new Module[4];
  AHRS gyro;
  SwerveDrivePoseEstimator poseEst;
  SwerveDriveKinematics kinematics;
  Pose2d startPose = new Pose2d(0, 0, new Rotation2d());
  double accelLim = SwerveConstants.accelLim;
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

  private final SysIdRoutine sysId;

  Field2d field = new Field2d();

  SwerveState current = SwerveState.NORMAL;

  /** Creates a new Swerve. */
  public Swerve(DoubleSupplier elevatorHeight) {
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

    this.elevatorHeight = elevatorHeight;

    poseEst =
        new SwerveDrivePoseEstimator(
            kinematics, startPose.getRotation(), getModulePostions(), startPose);

    SmartDashboard.putData(
        "Swerve/States", builder -> swerveStatesBuild(builder, this::getModuleStates));
    SmartDashboard.putData(
        "Swerve/Setpoints", builder -> swerveStatesBuild(builder, this::getModuleSetpoints));
    SmartDashboard.putData(
        "Swerve/Gyro",
        builder -> {
          builder.setSmartDashboardType("Gyro");
          builder.addDoubleProperty("Value", () -> getPose().getRotation().getDegrees(), null);
        });

    SmartDashboard.putData("Swerve/Field", field);
    SmartDashboard.putNumber("Tuning/Swerve/Velocity Setpoint", 0);
    SmartDashboard.putNumber("Tuning/Swerve/Angle Setpoint", 0);

    SmartDashboard.putNumber("Tuning/Swerve/Traj Translate P", SwerveConstants.translateP);
    SmartDashboard.putNumber("Tuning/Swerve/Traj Translate I", SwerveConstants.translateI);
    SmartDashboard.putNumber("Tuning/Swerve/Traj Translate D", SwerveConstants.translateD);

    SmartDashboard.putNumber("Tuning/Swerve/Traj Rotate P", SwerveConstants.rotateP);
    SmartDashboard.putNumber("Tuning/Swerve/Traj Rotate I", SwerveConstants.rotateI);
    SmartDashboard.putNumber("Tuning/Swerve/Traj Rotate D", SwerveConstants.rotateD);

    SmartDashboard.putNumber("Trajectory/XError", 0);
    SmartDashboard.putNumber("Trajectory/YError", 0);
    SmartDashboard.putNumber("Trajectory/HeadingError", 0);

    SmartDashboard.putNumber(
        "ElevatorSlowCoefficient", getCurrentSlowModeCoefficient(elevatorHeight.getAsDouble()));

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volt.of(4),
                Seconds.of(4),
                state -> SmartDashboard.putString("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                this::driveVoltage,
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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    poseEst.updateWithTime(
        RobotController.getFPGATime() * MICROS_SECONDS_CONVERSION,
        getGyroAngle(),
        getModulePostions());
    field.setRobotPose(getPose());

    SmartDashboard.putBoolean("Align Mode", current == SwerveState.LINEUP);

    sendDiagnostics();
  }

  public Pose2d getPose() {
    return poseEst.getEstimatedPosition();
  }

  public SwerveModulePosition[] getModulePostions() {
    return Stream.of(modules).map(Module::getPosition).toArray(SwerveModulePosition[]::new);
  }

  public SwerveModuleState[] getModuleStates() {
    return Stream.of(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
  }

  public SwerveModuleState[] getModuleSetpoints() {
    return Stream.of(modules).map(Module::getSetpoint).toArray(SwerveModuleState[]::new);
  }

  public Command readAngleEncoders() {
    return Commands.runOnce(
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
    return Commands.run(
            () ->
                drive(
                    squaredCurve(x.getAsDouble()),
                    squaredCurve(y.getAsDouble()),
                    omega.getAsDouble()),
            this)
        .withName("Swerve Drive Command");
  }

  public double squaredCurve(double input) {
    return Math.pow(input, 2) * Math.signum(input);
  }

  public Command pidTuningJogDrive() {
    return new RunCommand(
        () -> {
          SwerveModuleState state =
              new SwerveModuleState(
                  SmartDashboard.getNumber("Tuning/Swerve/Velocity Setpoint", 0), new Rotation2d());
          for (Module m : modules) {
            m.setModuleState(state, false);
          }
        },
        this);
  }

  public Command pidTuningJogAngle() {
    SwerveModuleState state =
        new SwerveModuleState(
            0,
            Rotation2d.fromDegrees(SmartDashboard.getNumber("Tuning/Swerve/Angle Setpoint", 0))
                .plus(modules[0].getAnglePosition()));
    return new RunCommand(
        () -> {
          for (Module m : modules) {
            m.setModuleState(state, false);
          }
        },
        this);
  }

  public Command pointWheelsForward() {
    return new RunCommand(
        () -> {
          for (Module m : modules) {
            m.setModuleState(new SwerveModuleState(0, new Rotation2d()), false);
          }
        },
        this);
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
    double slowModeYCoefficient;
    double slowModeXCoefficient;
    if (current == SwerveState.NORMAL) {
      slowModeYCoefficient = getCurrentSlowModeCoefficient(elevatorHeight.getAsDouble());
      slowModeXCoefficient = getCurrentSlowModeCoefficient(elevatorHeight.getAsDouble());

    } else {
      slowModeYCoefficient = 0.3;
      slowModeXCoefficient = 0.15;
    }

    y *= slowModeYCoefficient;
    x *= slowModeXCoefficient;

    // x = xLim.calculate(x);
    // y = yLim.calculate(y);
    // TODO: Reenable if wheelieing

    ChassisSpeeds chassisSpeeds;

    if (current == SwerveState.NORMAL) {
      // Takes in Alliance Relative, returns Field Relative
      chassisSpeeds = fromAllianceRelativeSpeeds(x, y, omega);
    } else {
      // Takes in Robot Relative, returns Robot Relative

      // Only allow driving on axes
      // if (Math.abs(x) >= Math.abs(y)) {
      //   y = 0;
      // } else {
      //   x = 0;
      // }

      // chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(y, -x, omega, getGyroAngle());
      chassisSpeeds = new ChassisSpeeds(y, x, omega);
    }

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

  public SendableBuilder swerveStatesBuild(
      SendableBuilder builder, Supplier<SwerveModuleState[]> states) {
    builder.setSmartDashboardType("SwerveDrive");

    builder.addDoubleProperty("Front Left Angle", () -> states.get()[0].angle.getDegrees(), null);
    builder.addDoubleProperty("Front Left Speed", () -> states.get()[0].speedMetersPerSecond, null);

    builder.addDoubleProperty("Front Right Angle", () -> states.get()[1].angle.getDegrees(), null);
    builder.addDoubleProperty(
        "Front Right Speed", () -> states.get()[1].speedMetersPerSecond, null);

    builder.addDoubleProperty("Back Left Angle", () -> states.get()[2].angle.getDegrees(), null);
    builder.addDoubleProperty("Back Left Speed", () -> states.get()[2].speedMetersPerSecond, null);

    builder.addDoubleProperty("Back Right Angle", () -> states.get()[3].angle.getDegrees(), null);
    builder.addDoubleProperty("Back Right Speed", () -> states.get()[3].speedMetersPerSecond, null);

    builder.addDoubleProperty("Robot Angle", () -> getPose().getRotation().getDegrees(), null);
    return builder;
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

    Translation2d allianceRelativeSpeeds = new Translation2d(arx, ary);

    if (DriverStation.getAlliance().isPresent()) {
      isRedAlliance = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    } else {
      DriverStation.reportError("No Alliance Present, Defaulting to RED", false);
    }

    if (isRedAlliance) { // RED ALLIANCE CASE
    } else { // BLUE ALLIANCE CASE
      allianceRelativeSpeeds = allianceRelativeSpeeds.rotateBy(Rotation2d.fromDegrees(180));
    }

    // Convert to Field Relative
    Translation2d fieldRelativeSpeeds =
        new Translation2d(-allianceRelativeSpeeds.getY(), -allianceRelativeSpeeds.getX());

    fr = new ChassisSpeeds(fieldRelativeSpeeds.getX(), fieldRelativeSpeeds.getY(), rot);
    fr = ChassisSpeeds.fromFieldRelativeSpeeds(fr, getPose().getRotation());

    return fr;
  }

  public double getCurrentSlowModeCoefficient(double elevatorHeight) {
    /* 0 to 1 value representing elevator position (0 is bottom, 1 is top) */
    double elevatorHeightPercent = elevatorHeight / ElevatorConstants.forwardSoftLimit;

    /* Don't limit at all if below some threshold */
    if (elevatorHeightPercent >= MIN_HEIGHT_PERCENTAGE_TO_LIMIT_SPEED) {

      /*
       * Scale slow mode position based on height percent using a parabola
       * y=\left\{0\le x\le h:1,h\le x\le1:\frac{l-1}{\left(1-h\right)^{2}}\left(x-h\right)^{2}+1\right\}
       * where h = MIN_HEIGHT_PERCENTAGE_TO_LIMIT_SPEED
       * & l = MAX_SLOW_MODE
       */
      double out =
          (MAX_SLOW_MODE - 1)
                  * Math.pow(elevatorHeightPercent - MIN_HEIGHT_PERCENTAGE_TO_LIMIT_SPEED, 2)
                  / Math.pow(1 - MIN_HEIGHT_PERCENTAGE_TO_LIMIT_SPEED, 2)
              + 1;
      SmartDashboard.putNumber("ElevatorSlowCoefficient", out);

      return out;
    }

    return 1;
  }

  public Command enableSlowMode() {
    return Commands.runOnce(() -> current = SwerveState.LINEUP);
  }

  public Command disableSlowMode() {
    return Commands.runOnce(() -> current = SwerveState.NORMAL);
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

    sendDiagnostics();
    // Apply the generated speeds
    driveChassisSpeedsRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation()));
  }

  public void sendDiagnostics() {
    for (Module m : modules) {
      SmartDashboard.putNumber(
          "ModuleDebug/Module" + m.getModuleNumber() + "FFoutput", m.getFFDriveOutput());
      SmartDashboard.putNumber(
          "ModuleDebug/Module" + m.getModuleNumber() + "AngleMotorOutput",
          m.getAppliedOutputAngle());
      SmartDashboard.putNumber(
          "ModuleDebug/Module" + m.getModuleNumber() + "Velocity", m.getDriveVelocity());
      SmartDashboard.putNumber(
          "ModuleDebug/Module" + m.getModuleNumber() + "Velocity Setpoint",
          m.getSetpoint().speedMetersPerSecond);

      SmartDashboard.putNumber(
          "SwerveSlowCoeff", getCurrentSlowModeCoefficient(elevatorHeight.getAsDouble()));
    }
  }

  public Command resetGyro() {
    return Commands.runOnce(
            () ->
                poseEst.resetPosition(
                    getGyroAngle(),
                    getModulePostions(),
                    new Pose2d(
                        poseEst.getEstimatedPosition().getX(),
                        poseEst.getEstimatedPosition().getY(),
                        new Rotation2d())))
        .ignoringDisable(true);
  }

  public void updateTrajectoryPID() {
    SwerveConstants.translateP =
        SmartDashboard.getNumber("Tuning/Swerve/Traj Translate P", SwerveConstants.translateP);
    SwerveConstants.translateI =
        SmartDashboard.getNumber("Tuning/Swerve/Traj Translate I", SwerveConstants.translateI);
    SwerveConstants.translateD =
        SmartDashboard.getNumber("Tuning/Swerve/Traj Translate D", SwerveConstants.translateD);

    SwerveConstants.rotateP =
        SmartDashboard.getNumber("Tuning/Swerve/Traj Rotate P", SwerveConstants.rotateP);
    SwerveConstants.rotateI =
        SmartDashboard.getNumber("Tuning/Swerve/Traj Rotate I", SwerveConstants.rotateI);
    SwerveConstants.rotateD =
        SmartDashboard.getNumber("Tuning/Swerve/Traj Rotate D", SwerveConstants.rotateD);
  }

  public Command resetOdometry() {
    return Commands.runOnce(
        () -> {
          poseEst.resetTranslation(new Translation2d());
          resetGyro();
        });
  }

  public void updateDashboardGUI() {
    Module m = modules[0];

    SmartDashboard.putNumber("Tuning/Swerve/Angle P", m.getAngleP());
    SmartDashboard.putNumber("Tuning/Swerve/Angle I", m.getAngleI());
    SmartDashboard.putNumber("Tuning/Swerve/Angle D", m.getAngleD());

    SmartDashboard.putNumber("Tuning/Swerve/Drive P", m.getDriveP());
    SmartDashboard.putNumber("Tuning/Swerve/Drive I", m.getDriveI());
    SmartDashboard.putNumber("Tuning/Swerve/Drive D", m.getDriveD());

    SmartDashboard.putNumber("Tuning/Swerve/Drive S", m.getDriveS());
    SmartDashboard.putNumber("Tuning/Swerve/Drive V", m.getDriveV());
    SmartDashboard.putNumber("Tuning/Swerve/Drive A", m.getDriveA());
  }

  public void updateControlConstants() {
    double[] drive = {
      SmartDashboard.getNumber("Tuning/Swerve/Drive P", ModuleConstants.driveP),
      SmartDashboard.getNumber("Tuning/Swerve/Drive I", ModuleConstants.driveI),
      SmartDashboard.getNumber("Tuning/Swerve/Drive D", ModuleConstants.driveD),
      SmartDashboard.getNumber("Tuning/Swerve/Drive S", ModuleConstants.driveS),
      SmartDashboard.getNumber("Tuning/Swerve/Drive V", ModuleConstants.driveV),
      SmartDashboard.getNumber("Tuning/Swerve/Drive A", ModuleConstants.driveA),
    };

    double[] angle = {
      SmartDashboard.getNumber("Tuning/Swerve/Angle P", ModuleConstants.angleP),
      SmartDashboard.getNumber("Tuning/Swerve/Angle I", ModuleConstants.angleI),
      SmartDashboard.getNumber("Tuning/Swerve/Angle D", ModuleConstants.angleD)
    };

    for (Module m : modules) {
      m.setNewControlConstants(drive, angle);
    }

    updateDashboardGUI();
  }
}
