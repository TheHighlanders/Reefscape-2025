// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.utils.StateHandler;
import frc.robot.utils.StateManagedSubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

class SwerveConstants {

  public static final double maxRotSpeed = Units.degreesToRadians(180);
  // Implicit /sec

  static double width = Units.inchesToMeters(20.5);
  static double length = width;

  static double maxSpeed = Constants.maxSpeed;
  // Implict /sec

  static double accelLim = 1.5;
}

public class Swerve extends StateManagedSubsystemBase<Swerve.SwerveState> {

  // This is the state handler bit for consistency purposes
  // <------------ State Handler Stuff ------------>
  enum SwerveState {
    FAST,
    SLOW
  }

  private static final double SLOW_MODE_MULTIPLIER = 0.3;
  private final StateHandler<SwerveState> stateHandler;
  // <------------ Non State Handler Stuff ------------>

  Module[] modules = new Module[4];
  AHRS gyro;
  SwerveDrivePoseEstimator poseEst;
  SwerveDriveKinematics kinematics;
  Pose2d startPose = new Pose2d(0, 0, new Rotation2d());

  StructArrayPublisher<SwerveModuleState> statePublisher;
  StructArrayPublisher<SwerveModuleState> setpointPublisher;
  StructPublisher<Pose2d> posePublisher;

  SlewRateLimiter xLim = new SlewRateLimiter(SwerveConstants.accelLim);
  SlewRateLimiter yLim = new SlewRateLimiter(SwerveConstants.accelLim);

  private final SysIdRoutine sysId;

  /** Creates a new Swerve. */
  public Swerve() {

    stateHandler = new StateHandler<>(SwerveState.FAST);

    for (int i = 0; i < modules.length; i++) {
      modules[i] = new Module(i);
    }

    // Using +X as forward, and +Y as left, as per
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    // FL, FR, BL, BR
    double y = SwerveConstants.width / 2.0d;
    double x = SwerveConstants.length / 2.0d;

    kinematics = new SwerveDriveKinematics(
        new Translation2d(x, y),
        new Translation2d(x, -y),
        new Translation2d(-x, y),
        new Translation2d(-x, -y));

    // Default Port is MXP
    gyro = new AHRS(NavXComType.kMXP_SPI);

    poseEst = new SwerveDrivePoseEstimator(
        kinematics,
        startPose.getRotation(),
        getModulePostions(),
        startPose);

    statePublisher = NetworkTableInstance
        .getDefault()
        .getStructArrayTopic("/Swerve/States", SwerveModuleState.struct)
        .publish();
    setpointPublisher = NetworkTableInstance
        .getDefault()
        .getStructArrayTopic("/Swerve/Setpoints", SwerveModuleState.struct)
        .publish();

    posePublisher = NetworkTableInstance
        .getDefault()
        .getStructTopic("/Swerve/Poses", Pose2d.struct)
        .publish();

    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volt.of(4),
            Seconds.of(6),
            state -> {
              SmartDashboard.putString("Drive/SysIdState", state.toString());
            }),
        new SysIdRoutine.Mechanism(
            voltage -> {
              driveVoltage(voltage);
            },
            log -> {
              log
                  .motor("Front-Left")
                  .voltage(
                      Volts.of(
                          modules[0].getDriveVolts().in(Volts) *
                              RobotController.getBatteryVoltage()))
                  .linearPosition(Meters.of(modules[0].getDrivePosition()))
                  .linearVelocity(
                      MetersPerSecond.of(modules[0].getDriveVelocity()));
              log
                  .motor("Front-Right")
                  .voltage(
                      Volts.of(
                          modules[1].getDriveVolts().in(Volts) *
                              RobotController.getBatteryVoltage()))
                  .linearPosition(Meters.of(modules[1].getDrivePosition()))
                  .linearVelocity(
                      MetersPerSecond.of(modules[1].getDriveVelocity()));
            },
            this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    poseEst.updateWithTime(
        RobotController.getFPGATime() * Math.pow(10, 6),
        getGyroAngle(),
        getModulePostions());

    sendNT();
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

  /**
   * Returns a command that will execute a quasistatic test in the given
   * direction.
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
  public Command driveCMD(
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier omega) {

    return new RunCommand(
        () -> {
          double speedMultiplier = stateHandler.getSubsystemStates().currentState() == SwerveState.SLOW
              ? SLOW_MODE_MULTIPLIER
              : 1.0;
          drive(
              x.getAsDouble() * speedMultiplier,
              y.getAsDouble() * speedMultiplier,
              omega.getAsDouble() * speedMultiplier);
        },
        this).withName("Swerve Drive Command");
  }

  /**
   * Method to drive the robot
   * 
   * @param x     Alliance Relative X Speed, as defined above (m/s)
   * @param y     Alliance Relative Y Speed, as defined above (m/s)
   * @param omega Rotational Speed (rad/s)
   */
  public void drive(double x, double y, double omega) {
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html

    ChassisSpeeds chassisSpeeds = fromAllianceRelativeSpeeds(
        xLim.calculate(x),
        yLim.calculate(y),
        omega); // Takes in Alliance Relative, returns Field Relative

    chassisSpeeds.vxMetersPerSecond *= SwerveConstants.maxSpeed;
    chassisSpeeds.vyMetersPerSecond *= SwerveConstants.maxSpeed;
    chassisSpeeds.omegaRadiansPerSecond *= SwerveConstants.maxRotSpeed;

    // TODO: make 0.02 measured instead of a constant.
    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveConstants.maxSpeed);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setModuleState(targetStates[i], false);
    }
  }

  public void sendNT() {
    statePublisher.set(getModuleStates());
    setpointPublisher.set(getModuleSetpoints());
    posePublisher.set(poseEst.getEstimatedPosition());
    SmartDashboard.putNumber(
        "Hypot",
        Math.pow(poseEst.getEstimatedPosition().getX(), 2) +
            Math.pow(poseEst.getEstimatedPosition().getY(), 2));
  }

  /**
   * Create Field Relative IN CHASSIS SPEEDS COORD SYSTEM Chassis Speeds from
   * Alliance Relative desired speeds
   * 
   * @param arx Alliance relative desired X speed
   * @param ary Alliance relative desired Y speed
   * @param rot Rotation speed, direction does not differ between alliances
   * @return Field Relative Chassis Speeds
   */
  public ChassisSpeeds fromAllianceRelativeSpeeds(
      double arx,
      double ary,
      double rot) {
    boolean isRedAlliance = true;
    ChassisSpeeds fr; // Field Relative

    // Rotation2d deg180 = Rotation2d.fromDegrees(180);

    Translation2d allianceRelativeSpeeds = new Translation2d(arx, ary);

    Translation2d fieldRelativeSpeeds;

    if (DriverStation.getAlliance().isPresent()) {
      isRedAlliance = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    } else {
      DriverStation.reportError(
          "No Alliance Present, Defaulting to RED",
          false);
    }
    SmartDashboard.putNumber(
        "Alliance Relative X BEFORE",
        allianceRelativeSpeeds.getX());

    if (isRedAlliance) {
    } else { // RED ALLIANCE CASE //BLUE ALLIANCE CASE
      allianceRelativeSpeeds = new Translation2d(
          -allianceRelativeSpeeds.getX(),
          -allianceRelativeSpeeds.getY());
    }

    SmartDashboard.putNumber(
        "Alliance Relative X AFTER",
        allianceRelativeSpeeds.getX());

    // Convert Blue Alliance Relative to Field Relative
    // fieldRelativeSpeeds =
    // allianceRelativeSpeeds.rotateBy(Rotation2d.fromDegrees(90));
    fieldRelativeSpeeds = new Translation2d(
        allianceRelativeSpeeds.getY(),
        -allianceRelativeSpeeds.getX());

    fr = new ChassisSpeeds(
        fieldRelativeSpeeds.getX(),
        fieldRelativeSpeeds.getY(),
        rot);
    fr = ChassisSpeeds.fromFieldRelativeSpeeds(fr, getGyroAngle());

    return fr;
  }

  public void driveVoltage(Measure<VoltageUnit> voltage) {
    for (Module m : modules) {
      m.driveVolts(voltage);
    }
  }
}
