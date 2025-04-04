// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.ElevatorConstants.forwardSoftLimit;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.revrobotics.spark.SparkBase.ControlType;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants;

import choreo.trajectory.SwerveSample;

final class SwerveConstants {

  static final double fieldWidth = 8.05;

  public static final double maxRotSpeed = Units.degreesToRadians(360);
  // Implicit /sec

  static final double width = Units.inchesToMeters(20.5);
  static final double length = width;

  static final double maxSpeed = Constants.maxSpeed;
  // Implict /sec

  static final double accelLim = 3;

  static final double headingCorrectionDeadband = 0.05;

  static double headingCorrectionP = 0.5;
  static double headingCorrectionI = 0.05;
  static double headingCorrectionD = 0;

  static double translateP = 1.75;
  static double translateI = 0.05;
  static double translateD = 0;

  static double rotateP = 1.5;
  static double rotateI = 0;
  static double rotateD = 0.6;

  static double orbitP = 0.9; // 0.75
  static double orbitI = 0;
  static double orbitD = 0.01;

  static double orbitCosScalar = Math.cos(Units.degreesToRadians(-15));
  static double orbitSinScalar = Math.sin(Units.degreesToRadians(-15));

  static double preAutoWheelTolerance = 2.5; // deg

  static Pose2d redReefCenter = new Pose2d(13.0514923438, 4.0259, new Rotation2d());
  static Pose2d blueReefCenter = new Pose2d(4.49673265625, 4.0259, new Rotation2d());

  static final StructPublisher<Pose2d> REEF_POSE_PUBLISHER =
      NetworkTableInstance.getDefault()
          .getStructTopic("Swerve/OrbitTarget", Pose2d.struct)
          .publish();

  static {
    REEF_POSE_PUBLISHER.accept(blueReefCenter);
  }
}

public class Swerve extends SubsystemBase {

  enum SwerveState {
    NORMAL,
    LINEUP
  }

  boolean tele = false;

  StructPublisher<Pose2d> orbitPosePublisher =
      NetworkTableInstance.getDefault().getStructTopic("Swerve/Orbit", Pose2d.struct).publish();

  private static final double MAX_SLOW_MODE = 0.3;
  private static final double MICROS_SECONDS_CONVERSION = Math.pow(10, -6);

  private static final double MIN_HEIGHT_PERCENTAGE_TO_LIMIT_SPEED = 0.25;

  DoubleSupplier elevatorHeight;

  private final Set<Module> needZeroing = new HashSet<>();

  PIDController headingDeadbandController =
      new PIDController(
          SwerveConstants.headingCorrectionP,
          SwerveConstants.headingCorrectionI,
          SwerveConstants.headingCorrectionD);

  Module[] modules = new Module[4];
  AHRS gyro;
  SwerveDrivePoseEstimator poseEst;
  public SwerveDriveKinematics kinematics;
  Pose2d startPose = new Pose2d(0, 0, new Rotation2d());

  static Rotation2d rightStationRotation;
  static Rotation2d leftStationRotation;

  private final PIDController xController =
      new PIDController(
          SwerveConstants.translateP, SwerveConstants.translateI, SwerveConstants.translateD);
  private final PIDController yController =
      new PIDController(
          SwerveConstants.translateP, SwerveConstants.translateI, SwerveConstants.translateD);
  private final PIDController headingController =
      new PIDController(SwerveConstants.rotateP, SwerveConstants.rotateI, SwerveConstants.rotateD);

  private final PIDController orbitController =
      new PIDController(SwerveConstants.orbitP, SwerveConstants.orbitI, SwerveConstants.orbitD);

  @Logged(name = "orbitX_PID_Out")
  private double orbitX_PID_Out = 0;

  @Logged(name = "orbitY_PID_Out")
  private double orbitY_PID_Out = 0;

  @Logged(name = "orbitTheta_PID_Out")
  private double orbitControllerOutput;

  private final SysIdRoutine sysId;

  Field2d field = new Field2d();

  SwerveState current = SwerveState.NORMAL;

  private final Vision vision;
  private Pose3d cameraPose;

  @Logged(name = "rotationTarget")
  Rotation2d rotationTarget = new Rotation2d();

  private boolean visionEnabled = true;

  /** Creates a new Swerve. */
  public Swerve(Vision vision, DoubleSupplier elevatorHeight) {
    this.vision = vision;
    this.elevatorHeight = elevatorHeight;
    this.cameraPose = new Pose3d();

    for (int i = 0; i < modules.length; i++) {
      modules[i] = new Module(i);
      needZeroing.add(modules[i]);
    }

    leftStationRotation =  Rotation2d.fromDegrees(54);
    rightStationRotation = new Rotation2d(-leftStationRotation.getCos(), leftStationRotation.getSin());

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      rightStationRotation = leftStationRotation.rotateBy(Rotation2d.k180deg);
      leftStationRotation = rightStationRotation.rotateBy(Rotation2d.k180deg);
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
            kinematics,
            startPose.getRotation(),
            getModulePostions(),
            startPose,
            VecBuilder.fill(0.5, 0.5, 0.5), // State standard deviations
            VecBuilder.fill(0.9, 0.9, 0.4)); // Default vision standard deviations

    SmartDashboard.putData("Swerve/Field", field);

    headingController.enableContinuousInput(-Math.PI, Math.PI);
    orbitController.enableContinuousInput(-Math.PI, Math.PI);

    orbitController.setTolerance(Units.degreesToRadians(5));

    if (Constants.devMode) {
      // Only send lots of data via NT if in devMode
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
    }

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
    attemptZeroingAbsolute();

    this.setName("Drive");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update odometry with latest module positions and gyro data
    poseEst.updateWithTime(
        RobotController.getFPGATime() * MICROS_SECONDS_CONVERSION,
        getGyroAngle(),
        getModulePostions());

    if (VisionConstants.POSE_STRATEGY == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE) {
      this.vision.addHeadingDataToEstimators(poseEst.getEstimatedPosition().getRotation());
    }

    // Update visualization
    field.setRobotPose(getPose());

    if(visionEnabled){
      // Process vision data from all cameras
      updateVision();
    }

    // Display alignment mode status
    // SmartDashboard.putBoolean("Align Mode", current == SwerveState.LINEUP);

    // "module" - progamming lead Miles
    for (Module module : modules) {
      if (MathUtil.isNear(
          module.getAnglePosition().getDegrees(), module.getSetpoint().angle.getDegrees(), 2)) {
        module.angleController.setReference(
            module.getAnglePosition().getDegrees(), ControlType.kPosition);
      }
    }
    // Send diagnostic information
    // sendDiagnostics();
  }

  public void startTele() {
    tele = true;
    visionEnabled = true;
  }

  public Command disableVision(){
    return Commands.runOnce(()->visionEnabled = false);
  }

  /** Process vision data from all cameras and update pose estimation */
  private void updateVision() {
    // for (int i = 0; i < Vision.CAMERA_COUNT; i++) {
    //   // Get the best result from vision
    //   Optional<EstimatedRobotPose> estPose = vision.getEstimatedRobotPose(i);

    //   if (estPose.isPresent()) {
    //     // Store the camera pose for debugging
    //     cameraPose = estPose.get().estimatedPose;

    //     // Get standard deviations from the camera
    //     Matrix<N3, N1> stdDev = vision.getEstimationStdDev(i);

    //     // Apply additional scaling based on game state
    //     if (DriverStation.isAutonomous()) {
    //       // Increase uncertainty during auto
    //       stdDev = stdDev.times(2.0);
    //     }

    //     // Add measurement to pose estimator
    //     poseEst.addVisionMeasurement(
    //         estPose.get().estimatedPose.toPose2d(), estPose.get().timestampSeconds, stdDev);

    //     lastEstTimestamp = estPose.get().timestampSeconds;

    //     // Log vision data
    //     SmartDashboard.putNumber(
    //         "Vision/" + vision.getName() + "/Processing Delay",
    //         Timer.getFPGATimestamp() - lastEstTimestamp);

    //     SmartDashboard.putNumber(
    //         "Vision/" + vision.getName() + "/Target Count", estPose.get().targetsUsed.size());

    //     // Log whether this camera has a valid result
    //     SmartDashboard.putBoolean(
    //         "Vision/" + vision.getName() + "/Has Target", estPose.isPresent());

    //     // Log camera poses for debugging
    //     if (Constants.devMode) {
    //       SmartDashboard.putData("Vision/Camera Poses", new Field2d());
    //       // Add code to visualize camera poses on field
    //     }
    //   }
    // }

    Optional<EstimatedRobotPose> bestEst = vision.getEstimatedRobotPose();
    if (bestEst.isPresent()) {
          // Store the camera pose for debugging
          cameraPose = bestEst.get().estimatedPose;
  
          // Get standard deviations from the camera
          Matrix<N3, N1> stdDev = vision.getEstimationStdDev();
  
          // Apply additional scaling based on game state
          if (DriverStation.isAutonomous()) {
            // Increase uncertainty during auto
            stdDev = stdDev.times(2.0);
          }
  
          // Add measurement to pose estimator
          poseEst.addVisionMeasurement(
              bestEst.get().estimatedPose.toPose2d(), bestEst.get().timestampSeconds, stdDev);
  
          // Log whether this camera has a valid result
          SmartDashboard.putBoolean(
              "Vision/" + vision.getName() + "/Has Target", bestEst.isPresent());
  
          // Log camera poses for debugging
          if (Constants.devMode) {
            SmartDashboard.putData("Vision/Camera Poses", new Field2d());
            // Add code to visualize camera poses on field
          }
        }
  }

  /**
   * Get a specific camera's pose
   *
   * @param index The camera index
   * @return The camera's pose in field coordinates
   */
  public Pose3d getCameraPose() {
    return cameraPose;
  }

  @Logged(name = "Estimated Pose", importance = Importance.INFO)
  public Pose2d getPose() {
    return poseEst.getEstimatedPosition();
  }

  public SwerveModulePosition[] getModulePostions() {
    return Stream.of(modules).map(Module::getPosition).toArray(SwerveModulePosition[]::new);
  }

  public Command resetPoseToVision() {
    return Commands.runOnce(
        () -> {
          if (vision.getEstimatedRobotPose().isPresent()) {
            poseEst.resetPosition(
                getGyroAngle(),
                getModulePostions(),
                vision.getEstimatedRobotPose().get().estimatedPose.toPose2d());
          }
        });
  }

  @Logged(name = "Swerve Module States", importance = Importance.INFO)
  public SwerveModuleState[] getModuleStates() {
    return Stream.of(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
  }

  @Logged(name = "Swerve Module Setpoints", importance = Importance.INFO)
  public SwerveModuleState[] getModuleSetpoints() {
    return Stream.of(modules).map(Module::getSetpoint).toArray(SwerveModuleState[]::new);
  }

  @Logged(name = "Drive Applied Outputs", importance = Importance.INFO)
  public double[] getDriveAppliedOutputs() {
    double[] outputs = new double[4];

    for (int i = 0; i < outputs.length; i++) {
      outputs[i] = modules[i].getAppliedOutputDrive();
    }

    return outputs;
  }

  @Logged(name = "Drive Angle Errors", importance = Importance.INFO)
  public double[] getAngleErrors() {
    double[] out = new double[4];

    for (int i = 0; i < modules.length; i++) {
      out[i] =
          modules[i].getSetpoint().angle.getDegrees() - modules[i].getState().angle.getDegrees();
    }

    return out;
  }

  @Logged(name = "Angle Applied Outputs", importance = Importance.INFO)
  public double[] getAngleAppliedOutputs() {
    double[] outputs = new double[4];

    for (int i = 0; i < outputs.length; i++) {
      outputs[i] = modules[i].getAppliedOutputAngle();
    }

    return outputs;
  }

  public boolean attemptZeroingAbsolute() {
    if (!needZeroing.isEmpty()) {
      DriverStation.reportWarning(
          "ZEROING SOME CANCODERS FAILED, " + needZeroing.size() + " ARE BEING REZEROED", false);

      Iterator<Module> iterator = needZeroing.iterator();
      while (iterator.hasNext()) {
        Module module = iterator.next();
        module.resetAbsolute();
        iterator.remove();
      }

      return false;
    }
    return true;
  }

  public Command driveOrbit(DoubleSupplier x, DoubleSupplier y) {
    return Commands.run(
            () -> {
              rotationTarget =
                  new Rotation2d(
                          Math.atan2(
                              reefPose().getY() - getPose().getY(),
                              reefPose().getX() - getPose().getX()))
                      .plus(Rotation2d.fromDegrees(-15));

              orbitPosePublisher.accept(
                  new Pose2d(getPose().getX(), getPose().getY(), rotationTarget));

              if (current == SwerveState.LINEUP) {
                orbitX_PID_Out =
                    (x.getAsDouble() * SwerveConstants.orbitCosScalar)
                        - (y.getAsDouble() * SwerveConstants.orbitSinScalar);
                orbitY_PID_Out =
                    (y.getAsDouble() * SwerveConstants.orbitCosScalar)
                        + (x.getAsDouble()
                            * SwerveConstants
                                .orbitSinScalar); // rotates inputs by 15 degrees to accout for
                // offset for cameras
              } else {
                orbitX_PID_Out = x.getAsDouble();
                orbitY_PID_Out = y.getAsDouble();
              }

              double radiansOff =
                  getPose().getRotation().getRadians() - rotationTarget.getRadians();

              orbitControllerOutput = orbitController.calculate(radiansOff, 0);
              // SmartDashboard.putNumber("Orbit/Error", radiansOff);

              drive(
                  squaredCurve(orbitX_PID_Out),
                  squaredCurve(orbitY_PID_Out),
                  -orbitControllerOutput);
            },
            this)
        .withName("Orbit Drive Command");
  }

  public Command driveStation(DoubleSupplier x, DoubleSupplier y) {
    return Commands.run(
            () -> {
              rotationTarget =
                  getPose().getTranslation().getY() > SwerveConstants.fieldWidth / 2
                      ? leftStationRotation
                      : rightStationRotation;

              orbitPosePublisher.accept(
                  new Pose2d(getPose().getX(), getPose().getY(), rotationTarget));
              orbitX_PID_Out = x.getAsDouble();
              orbitY_PID_Out = y.getAsDouble();

              double radiansOff =
                  getPose().getRotation().getRadians() - rotationTarget.getRadians();

              orbitControllerOutput = orbitController.calculate(radiansOff, 0);

              drive(
                  squaredCurve(orbitX_PID_Out),
                  squaredCurve(orbitY_PID_Out),
                  -orbitControllerOutput);
            },
            this)
        .withName("Orbit Drive Command");
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
        .ignoringDisable(true)
        .withName("Read Angle Encoders");
  }

  @Logged(name = "Gyro", importance = Importance.INFO)
  public Rotation2d getGyroAngle() {
    return gyro.getRotation2d();
  }

  public void resetOdometry(Pose2d pose) {
    poseEst.resetPosition(getGyroAngle(), getModulePostions(), pose);
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction).withName("SysId Quasistatic " + direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction).withName("SysId Dynamic " + direction);
  }

  public Command sysId() {
    return Commands.sequence(
            sysIdDynamic(Direction.kForward).withName("Dynamic Forward"),
            Commands.waitSeconds(1).withName("Wait 1"),
            sysIdDynamic(Direction.kReverse).withName("Dynamic Reverse"),
            Commands.waitSeconds(1).withName("Wait 2"),
            sysIdQuasistatic(Direction.kForward).withName("Quasistatic Forward"),
            Commands.waitSeconds(1).withName("Wait 3"),
            sysIdQuasistatic(Direction.kReverse).withName("Quasistatic Reverse"))
        .withName("Complete SysId Sequence");
  }

  public Pose2d reefPose() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      return SwerveConstants.redReefCenter;
    }
    return SwerveConstants.blueReefCenter;
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

  public Command driveRobotRelativeCMD(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return Commands.run(
        () -> {
          ChassisSpeeds speeds =
              new ChassisSpeeds(x.getAsDouble(), y.getAsDouble(), omega.getAsDouble());
          driveChassisSpeedsRobotRelative(speeds);
        });
  }

  public double squaredCurve(double input) {
    return Math.pow(input, 2) * Math.signum(input);
  }

  public Command pidTuningJogDrive() {
    return new RunCommand(
            () -> {
              SwerveModuleState state =
                  new SwerveModuleState(
                      SmartDashboard.getNumber("Tuning/Swerve/Velocity Setpoint", 0),
                      new Rotation2d());
              for (Module m : modules) {
                m.setModuleState(state, false);
              }
            },
            this)
        .withName("PID Tuning Jog Drive");
  }

  /**
   * Points the wheels in an X pattern to lock the robot in place
   *
   * @return Command to set modules in X pattern
   */
  public Command pointWheelsInXPattern() {
    return new RunCommand(
            () -> {
              SwerveModuleState[] states = new SwerveModuleState[4];

              // FL at 45°, FR at 135°, BL at 315° (-45°), BR at 225° (-135°)
              states[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45)); // Front Left
              states[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(135)); // Front Right
              states[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(315)); // Back Left
              states[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(225)); // Back Right

              for (int i = 0; i < modules.length; i++) {
                modules[i].setModuleState(states[i], false);
              }
            },
            this)
        .withName("Point Wheels in X Pattern");
  }

  public Command driveForwardTimed(double velocity, double timeSec) {
    return new RunCommand(
            () -> {
              SwerveModuleState state =
                  new SwerveModuleState(velocity, Rotation2d.fromDegrees(180));
              for (Module m : modules) {
                m.setModuleState(state, false);
              }
            },
            this)
        .withTimeout(timeSec)
        .finallyDo(this::stopDrive)
        .withName("Drive Forward Timed " + velocity + "m/s for " + timeSec + "s");
  }

  public void stopDrive() {
    SwerveModuleState state = new SwerveModuleState(0, new Rotation2d());
    for (Module m : modules) {
      m.setModuleState(state, false);
    }
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
            this)
        .withName("PID Tuning Jog Angle");
  }

  public Command pointWheelsForward() {
    return new RunCommand(
            () -> {
              for (Module m : modules) {
                m.setModuleState(new SwerveModuleState(0, new Rotation2d()), false);
              }
            },
            this)
        .withName("Point Wheels Forward");
  }

  public Command resetWheelsToZero() {
    return Commands.runOnce(
            () -> {
              for (Module m : modules) {
                m.angleEncoder.setPosition(0);
              }
            })
        .withName("Reset Wheels to Zero");
  }

  public Command presetWheelsToTraj(SwerveSample sample) {
    SwerveModuleState[] wheelDirections = new SwerveModuleState[4];

    SwerveModuleState[] wheelStates = kinematics.toSwerveModuleStates(sample.getChassisSpeeds());

    for (int i = 0; i < wheelStates.length; i++) {
      wheelDirections[i] = new SwerveModuleState(0, wheelStates[i].angle);
    }

    return Commands.sequence(
            Commands.runOnce(
                    () -> {
                      for (int i = 0; i < modules.length; i++) {
                        wheelDirections[i].optimize(getModulePostions()[i].angle);
                        modules[i].setModuleState(wheelDirections[i], false);
                      }
                    })
                .withName("Set Wheel Directions"),
            Commands.waitUntil(() -> areModulesAtAngleSetpoint(wheelDirections))
                .withName("Wait for Modules at Angle"))
        .withName("Preset Wheels to Trajectory");
  }

  public boolean areModulesAtAngleSetpoint(SwerveModuleState[] directions) {
    for (int i = 0; i < directions.length; i++) {
      if (!MathUtil.isNear(
          directions[i].angle.getDegrees(),
          modules[i].getAnglePosition().getDegrees(),
          SwerveConstants.preAutoWheelTolerance)) {
        return false;
      }
    }

    return true;
  }

  /**
   * If the robot isnt commanded to rotate and we are moving will attempt to keep the robot at its
   * current roation
   *
   * @param x Alliance Relative X Speed, as defined above (m/s)
   * @param y Alliance Relative Y Speed, as defined above (m/s)
   * @param omega Rotational Speed (rad/s)
   * @return the desired rotational speed of the robot
   */
  public double headingCorrection(double x, double y, double omega) {
    if (MathUtil.isNear(0, omega, SwerveConstants.headingCorrectionDeadband)
        && Math.max(Math.abs(x), Math.abs(y)) > SwerveConstants.headingCorrectionDeadband) {
      return headingDeadbandController.calculate(Units.degreesToRadians(gyro.getRate()), 0);
    }
    return omega;
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
      slowModeYCoefficient = 0.2;
      slowModeXCoefficient = 0.1;
    }

    y *= slowModeYCoefficient;
    x *= slowModeXCoefficient;

    // Comment to disable heading correction
    // omega = headingCorrection(x, y, omega);
    ChassisSpeeds chassisSpeeds;

    if (current == SwerveState.NORMAL) {
      // Takes in Alliance Relative, returns Field Relative
      chassisSpeeds = fromAllianceRelativeSpeeds(x, y, omega);
    } else {
      // Takes in Robot Relative, returns Robot Relative

      // Only allow driving on axes
      // if (Math.abs(x) >= Math.abs(y)) {
      // y = 0;
      // } else {
      // x = 0;
      // }
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
      targetStates[i].cosineScale(getModulePostions()[i].angle);
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
    double elevatorHeightPercent = elevatorHeight / forwardSoftLimit;

    /* Don't limit at all if below some threshold */
    if (elevatorHeightPercent >= MIN_HEIGHT_PERCENTAGE_TO_LIMIT_SPEED) {

      /*
       * Scale slow mode position based on height percent using a parabola
       * y=\left\{0\le x\le h:1,h\le
       * x\le1:\frac{l-1}{\left(1-h\right)^{2}}\left(x-h\right)^{2}+1\right\}
       * where h = MIN_HEIGHT_PERCENTAGE_TO_LIMIT_SPEED
       * & l = MAX_SLOW_MODE
       */
      double out =
          (MAX_SLOW_MODE - 1)
                  * Math.pow(elevatorHeightPercent - MIN_HEIGHT_PERCENTAGE_TO_LIMIT_SPEED, 2)
                  / Math.pow(1 - MIN_HEIGHT_PERCENTAGE_TO_LIMIT_SPEED, 2)
              + 1;

      if (Constants.devMode) {
        SmartDashboard.putNumber("ElevatorSlowCoefficient", out);
      }

      return out;
    }

    return 1;
  }

  public Command enableSlowMode() {
    return Commands.runOnce(() -> current = SwerveState.LINEUP).withName("Enable Slow Mode");
  }

  public Command toggleSlowMode() {
    return Commands.startEnd(() -> current = SwerveState.LINEUP, () -> current = SwerveState.NORMAL)
        .withName("Enable Slow Mode");
  }

  public Command disableSlowMode() {
    return Commands.runOnce(() -> current = SwerveState.NORMAL).withName("Disable Slow Mode");
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

    if(Constants.devMode){
    SmartDashboard.putNumber("Trajectory/XError", xController.getError());
    SmartDashboard.putNumber("Trajectory/YError", yController.getError());
    SmartDashboard.putNumber("Trajectory/HeadingError", headingController.getError());
    }

    // sendDiagnostics();
    // Apply the generated speeds
    driveChassisSpeedsRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation()));
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
        .ignoringDisable(true)
        .withName("Reset Gyro");
  }

  public Command resetOdometry() {
    return Commands.runOnce(
            () -> {
              poseEst.resetTranslation(new Translation2d());
              resetGyro();
            })
        .withName("Reset Odometry");
  }

  public void sendDiagnostics() {
    for (Module m : modules) {
      SmartDashboard.putNumber(
          "ModuleDebug/Module" + m.getModuleNumber() + "FFoutput", m.getFFDriveOutput());
      SmartDashboard.putNumber(
          "ModuleDebug/Module" + m.getModuleNumber() + "DriveMotorOutput",
          m.getAppliedOutputDrive());
      SmartDashboard.putNumber(
          "ModuleDebug/Module" + m.getModuleNumber() + "Velocity", m.getDriveVelocity());
      SmartDashboard.putNumber(
          "ModuleDebug/Module" + m.getModuleNumber() + "Velocity Setpoint",
          m.getSetpoint().speedMetersPerSecond);
      SmartDashboard.putNumber(
          "ModuleDebug/Module" + m.getModuleNumber() + "Output Percentage Angle",
          m.getAppliedOutputAngle());
      SmartDashboard.putNumber(
          "ModuleDebug/Module" + m.getModuleNumber() + "Output Percentage Drive",
          m.getAppliedOutputDrive());

      SmartDashboard.putNumber(
          "ModuleDebug/Module" + m.getModuleNumber() + "Position", m.getPosition().distanceMeters);

      SmartDashboard.putNumber(
          "ModuleDebug/ModuleRelative" + m.getModuleNumber() + "Position",
          m.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "ModuleDebug/ModuleAbsolute" + m.getModuleNumber() + "Position",
          m.getAbsolutePosition().getDegrees());

      SmartDashboard.putNumber(
          "SwerveSlowCoeff", getCurrentSlowModeCoefficient(elevatorHeight.getAsDouble()));
    }
  }

  public void updateTrajectoryPID() {
    xController.setP(
        SmartDashboard.getNumber("Tuning/Swerve/Traj Translate P", SwerveConstants.translateP));
    yController.setP(
        SmartDashboard.getNumber("Tuning/Swerve/Traj Translate P", SwerveConstants.translateP));

    xController.setI(
        SmartDashboard.getNumber("Tuning/Swerve/Traj Translate I", SwerveConstants.translateI));
    yController.setI(
        SmartDashboard.getNumber("Tuning/Swerve/Traj Translate I", SwerveConstants.translateI));

    xController.setD(
        SmartDashboard.getNumber("Tuning/Swerve/Traj Translate D", SwerveConstants.translateD));
    yController.setD(
        SmartDashboard.getNumber("Tuning/Swerve/Traj Translate D", SwerveConstants.translateD));

    headingController.setP(
        SmartDashboard.getNumber("Tuning/Swerve/Traj Rotate P", SwerveConstants.rotateP));
    headingController.setI(
        SmartDashboard.getNumber("Tuning/Swerve/Traj Rotate I", SwerveConstants.rotateI));
    headingController.setD(
        SmartDashboard.getNumber("Tuning/Swerve/Traj Rotate D", SwerveConstants.rotateD));
  }

  public void updateDashboardGUI() {
    Module m = modules[0];

    SmartDashboard.putNumber("Tuning/Swerve/Correction P", headingDeadbandController.getP());
    SmartDashboard.putNumber("Tuning/Swerve/Correction I", headingDeadbandController.getI());
    SmartDashboard.putNumber("Tuning/Swerve/Correction D", headingDeadbandController.getD());

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

    double[] correction = {
      SmartDashboard.getNumber("Tuning/Swerve/Correction P", SwerveConstants.headingCorrectionP),
      SmartDashboard.getNumber("Tuning/Swerve/Correction I", SwerveConstants.headingCorrectionI),
      SmartDashboard.getNumber("Tuning/Swerve/Correction D", SwerveConstants.headingCorrectionD)
    };

    for (Module m : modules) {
      m.setNewControlConstants(drive, angle);
    }

    headingDeadbandController.setPID(correction[0], correction[1], correction[2]);

    updateDashboardGUI();
  }
}
