package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

final class ModuleConstants {
  static final double angleP = 0.25;
  static final double angleI = 0.0;
  static final double angleD = 0;

  static final double driveP = 0.2;
  static final double driveI = 0;
  static final double driveD = 0;

  static final double driveS = 0;
  static final double driveV = 3.5;
  static final double driveA = 0;

  // Wheel diameter * pi / gear ratio
  static final double drivePCF = inchesToMeters(3 + 13d / 16d) * Math.PI / 6.75d;

  static final double anglePCF = 360.0 / 12.8d;

  static final int driveCurrentLimit = 40;
  static final int angleCurrentLimit = 25;

  static final boolean absolInverted = false;

  static final double maxSpeed = Constants.maxSpeed;
}

public class Module {

  public SparkMax angleMotor;
  public SparkMax driveMotor;

  public SparkClosedLoopController driveController;
  public SparkClosedLoopController angleController;

  public int moduleNumber;

  public RelativeEncoder driveEncoder;
  public RelativeEncoder angleEncoder;
  public double angleReference;
  public double driveReference;

  public CANcoder absoluteEncoder;
  public StatusSignal<Angle> absoluteEncoderPosition;

  public boolean hasZeroedAbsolute = false;

  /* Creates an additional FF controller for extra drive motor control */
  private static SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(
          ModuleConstants.driveS, ModuleConstants.driveV, ModuleConstants.driveA);

  double ffOut = 0;

  public Module(int moduleNumber) {
    this.moduleNumber = moduleNumber;

    driveMotor = new SparkMax(Constants.driveMotorIDs[moduleNumber], MotorType.kBrushless);
    angleMotor = new SparkMax(Constants.angleMotorIDs[moduleNumber], MotorType.kBrushless);

    SparkMaxConfig driveConfig = createDriveConfig();
    SparkMaxConfig angleConfig = createAngleConfig();

    angleMotor.configure(
        angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveMotor.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveEncoder = driveMotor.getEncoder();
    angleEncoder = angleMotor.getEncoder();

    absoluteEncoder = new CANcoder(Constants.canCoderIDs[moduleNumber]);
    absoluteEncoderPosition = absoluteEncoder.getPosition();

    angleController = angleMotor.getClosedLoopController();
    driveController = driveMotor.getClosedLoopController();

    driveEncoder.setPosition(0);

    // Zero Angle Encoder (NEO internal) against CANcoder
    resetAbsolute();
  }

  private SparkMaxConfig createDriveConfig() {
    SparkMaxConfig driveConfig = new SparkMaxConfig();

    driveConfig.inverted(false);

    driveConfig
        .encoder
        .positionConversionFactor(ModuleConstants.drivePCF)
        .velocityConversionFactor(ModuleConstants.drivePCF / 60.0d);

    driveConfig.closedLoop.pid(
        ModuleConstants.driveP, ModuleConstants.driveI, ModuleConstants.driveD);

    driveConfig.smartCurrentLimit(ModuleConstants.driveCurrentLimit).idleMode(IdleMode.kBrake);

    return driveConfig;
  }

  private SparkMaxConfig createAngleConfig() {
    SparkMaxConfig angleConfig = new SparkMaxConfig();

    angleConfig.inverted(false);

    angleConfig
        .encoder
        .positionConversionFactor(ModuleConstants.anglePCF)
        .velocityConversionFactor(ModuleConstants.anglePCF / 60.0d);

    angleConfig
        .closedLoop
        .pid(ModuleConstants.angleP, ModuleConstants.angleI, ModuleConstants.angleD)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-180.0d, 180.0d)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    angleConfig.smartCurrentLimit(ModuleConstants.angleCurrentLimit).idleMode(IdleMode.kCoast);

    return angleConfig;
  }

  /**
   * Sets both Angle and Drive to desired states
   *
   * @param state: Desired module state
   * @param isOpenLoop: Controls if the drive motor use a PID loop
   */
  public void setModuleState(SwerveModuleState state, boolean isOpenLoop) {
    setAngleState(state);
    setDriveState(state, isOpenLoop);
  }

  /**
   * Sets the Drive Motor to a desired state, if isOpenLoop is true, it will be set as a percent, if
   * it is false, than it will use a velocity PIDF loop
   *
   * @param state: Desired module state
   * @param isOpenLoop: Whether or not to use a PID loop
   */
  public void setDriveState(SwerveModuleState state, boolean isOpenLoop) {
    // double angleToSetDeg = state.angle.getRotations();
    double velocityToSet = state.speedMetersPerSecond;

    // /*
    //  * From FRC 900's whitepaper, we add a cosine compensator to the applied drive
    //  * velocity
    //  */
    // /* To reduce the "skew" that occurs when changing direction */
    // double steerMotorError = angleToSetDeg - (angleEncoder.getPosition() / 360.0d);
    // // If error is close to 0 rotations, we're already there, so apply full power
    // // If the error is close to 0.25 rotations, then we're 90 degrees, so movement
    // // doesn't help
    // // us at all
    // // We take the absolute value of this to make sure we don't invert our drive,
    // // even though we
    // // shouldn't ever target over 90 degrees anyway
    // double cosineScalar = Math.abs(Math.cos(Units.rotationsToRadians(steerMotorError)));
    // velocityToSet *= cosineScalar;

    if (isOpenLoop) {
      double motorPercent = state.speedMetersPerSecond / ModuleConstants.maxSpeed;
      driveMotor.set(motorPercent);
      driveReference = state.speedMetersPerSecond;
    } else {
      ffOut = driveFeedforward.calculate(velocityToSet);
      driveController.setReference(
          velocityToSet, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffOut);
      driveReference = velocityToSet;
    }
  }

  /**
   * Sets the Angle Motor to a desired state, does not set the state if speed is too low, to stop
   * wheel jitter
   *
   * @param state: Desired module state
   */
  public void setAngleState(SwerveModuleState state) {
    Rotation2d angle = state.angle;
    if (angle != null) {
      angleController.setReference(angle.getDegrees(), ControlType.kPosition);
      angleReference = angle.getDegrees();
    }

    if (Constants.devMode) {
      SmartDashboard.putNumber(
          "Tuning/Swerve/Angle/" + moduleNumber + "Angle Setpoint", angleReference);
    }
  }

  /**
   * Returns the position of the Angle Motor, measured with integrated encoder
   *
   * @return Angle Motor Position
   */
  public Rotation2d getAnglePosition() {
    return Rotation2d.fromDegrees(angleEncoder.getPosition());
  }

  /**
   * Returns the velocity of the Drive Motor, measured with integrated encoder
   *
   * @return Drive Motor Velocity
   */
  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getAppliedOutputDrive() {
    return driveMotor.getAppliedOutput();
  }

  public double getAppliedOutputAngle() {
    return angleMotor.getAppliedOutput();
  }

  public double getFFDriveOutput() {
    return ffOut;
  }

  /**
   * Gets the position of the Drive Motor, measured with integrated encoder
   *
   * @return Drive Motor Position
   */
  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  /**
   * Gets the position of the module using the absolute encoder
   *
   * @return Position of the module between 0 and 360, as a Rotation2d
   */
  public Rotation2d getAbsolutePosition() {
    /* Gets Position from CANcoder */
    if (BaseStatusSignal.isAllGood(absoluteEncoderPosition)) {
      return Rotation2d.fromDegrees(absoluteEncoderPosition.getValue().in(Degrees));
    } else if (hasZeroedAbsolute) {
      DriverStation.reportWarning(
          "CANCODER NOT GOOD DEFAULTING TO RELATIVE: MODULE " + moduleNumber, false);
      return Rotation2d.fromDegrees(angleEncoder.getPosition());
    } else {
      DriverStation.reportError(
          "CANCODER NOT GOOD AND MODULE NOT ZEROED: MODULE " + moduleNumber, false);
      return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }
  }

  /**
   * @return Swerve Module Position (Position & Angle)
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(-getDrivePosition(), getAnglePosition());
  }

  public void driveVolts(Measure<VoltageUnit> voltage) {
    setAngleState(new SwerveModuleState(0, new Rotation2d()));
    driveMotor.setVoltage(voltage.in(Volts));
  }

  public boolean resetAbsolute() {
    if (BaseStatusSignal.isAllGood(absoluteEncoderPosition)) {
      angleEncoder.setPosition(absoluteEncoderPosition.getValue().in(Degrees));
      hasZeroedAbsolute = true;
    }
    return hasZeroedAbsolute;
  }

  // TODO: Investigate
  public Measure<VoltageUnit> getDriveVolts() {
    return Volts.of(driveMotor.getBusVoltage() * driveMotor.getAppliedOutput());
  }

  /**
   * @return Swerve Module State (Velocity & Angle)
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getAnglePosition());
  }

  public SwerveModuleState getSetpoint() {
    return new SwerveModuleState(driveReference, Rotation2d.fromDegrees(angleReference));
  }

  /** Returns the assigned module number */
  public int getModuleNumber() {
    return moduleNumber;
  }

  public boolean getAngleInverted() {
    return angleMotor.configAccessor.getInverted();
  }

  public double getAngleP() {
    return angleMotor.configAccessor.closedLoop.getP();
  }

  public double getAngleI() {
    return angleMotor.configAccessor.closedLoop.getI();
  }

  public double getAngleD() {
    return angleMotor.configAccessor.closedLoop.getD();
  }

  public double getDriveP() {
    return driveMotor.configAccessor.closedLoop.getP();
  }

  public double getDriveI() {
    return driveMotor.configAccessor.closedLoop.getI();
  }

  public double getDriveD() {
    return driveMotor.configAccessor.closedLoop.getD();
  }

  public double getDriveS() {
    return driveFeedforward.getKs();
  }

  public double getDriveV() {
    return driveFeedforward.getKv();
  }

  public double getDriveA() {
    return driveFeedforward.getKa();
  }

  private static void updateDriveFeedforward(double s, double v, double a) {
    driveFeedforward = new SimpleMotorFeedforward(s, v, a);
  }

  public void setNewControlConstants(double[] drive, double[] angle) {
    updateDriveConstants(drive);
    updateAngleConstants(angle);

    updateDriveFeedforward(drive[3], drive[4], drive[5]);
  }

  public void updateDriveConstants(double[] drive) {
    ClosedLoopConfigAccessor config = driveMotor.configAccessor.closedLoop;

    double p = config.getP();
    double i = config.getI();
    double d = config.getD();

    if (drive[0] != p || drive[1] != i || drive[2] != d) {

      SparkMaxConfig newConfig = new SparkMaxConfig();
      newConfig.closedLoop.pid(drive[0], drive[1], drive[2]);
      driveMotor.configure(
          newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  public void updateAngleConstants(double[] angle) {
    ClosedLoopConfigAccessor config = angleMotor.configAccessor.closedLoop;

    double p = config.getP();
    double i = config.getI();
    double d = config.getD();

    if (angle[0] != p || angle[1] != i || angle[2] != d) {
      SparkMaxConfig newConfig = new SparkMaxConfig();
      newConfig.closedLoop.pid(angle[0], angle[1], angle[2]);
      angleMotor.configure(
          newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  }
}
