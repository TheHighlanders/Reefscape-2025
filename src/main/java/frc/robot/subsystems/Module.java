package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;

class moduleConstants {
  static double angleP = 0.05;
  static double angleI = 0;
  static double angleD = 0.002;

  static double driveP = 0.2; // 2.1301;//0.2;
  static double driveI = 0;
  static double driveD = 3; // 3;

  static double driveS = 0.1718; // 0.1718;//0.375;
  static double driveV = 4; // 3.2228;//2.5;
  static double driveA = 8; // 0.74971;//0;

  // TODO: Change to L2 (6.75) when we go to actual modules
  static double drivePCF =
      edu.wpi.first.math.util.Units.inchesToMeters(3 + 13d / 16d) * Math.PI / 8.14d;
  static double anglePCF = 360.0 / 12.8d;

  static int driveCurrentLimit = 40;
  static int angleCurrentLimit = 20;

  static boolean absolInverted = false;

  static double maxSpeed = Constants.maxSpeed;
}

public class Module {

  public SparkMax angleMotor;
  public SparkMax driveMotor;

  public SparkSim driveSim;
  public SparkSim angleSim;

  public DCMotorSim angleMotorSim;
  public DCMotorSim driveMotorSim;

  public SparkClosedLoopController driveController;
  public SparkClosedLoopController angleController;

  public int moduleNumber;

  private static SimpleMotorFeedforward driveFeedforward;

  public RelativeEncoder driveEncoder;
  public RelativeEncoder angleEncoder;
  public double angleReference;
  public double driveReference;

  public SparkAbsoluteEncoder absoluteEncoder;

  private Rotation2d KModuleAbsoluteOffset;

  DCMotor angleNeo;
  DCMotor driveNeo;

  DCMotorSim angleNeoSim;
  DCMotorSim driveNeoSim;

  double ffOut = 0;

  public Module(int moduleNumber) {
    this.KModuleAbsoluteOffset = Rotation2d.fromDegrees(Constants.absoluteOffsets[moduleNumber]);
    this.moduleNumber = moduleNumber;

    driveMotor = new SparkMax(Constants.driveMotorIDs[moduleNumber], MotorType.kBrushless);
    angleMotor = new SparkMax(Constants.angleMotorIDs[moduleNumber], MotorType.kBrushless);

    angleNeo = DCMotor.getNEO(1);
    driveNeo = DCMotor.getNEO(1);

    angleNeoSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(angleNeo, 0.004, moduleConstants.anglePCF),
            angleNeo);
    driveNeoSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(driveNeo, 0.025, 1 / 8.14), driveNeo);

    if (Constants.sim) {
      driveSim = new SparkSim(driveMotor, driveNeo);
      angleSim = new SparkSim(angleMotor, angleNeo);
    }

    SparkMaxConfig driveConfig = createDriveConfig();
    SparkMaxConfig angleConfig = createAngleConfig();

    /* Creates an additional FF controller for extra drive motor control */
    driveFeedforward =
        new SimpleMotorFeedforward(
            moduleConstants.driveS, moduleConstants.driveV, moduleConstants.driveA);

    angleMotor.configure(
        angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveMotor.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveEncoder = driveMotor.getEncoder();
    angleEncoder = angleMotor.getEncoder();

    absoluteEncoder = angleMotor.getAbsoluteEncoder();

    angleController = angleMotor.getClosedLoopController();
    driveController = driveMotor.getClosedLoopController();

    driveEncoder.setPosition(0);

    angleEncoder.setPosition(getAbsolutePosition().getDegrees());
  }

  private SparkMaxConfig createDriveConfig() {
    SparkMaxConfig driveConfig = new SparkMaxConfig();

    driveConfig.inverted(false);

    driveConfig
        .encoder
        .positionConversionFactor(moduleConstants.drivePCF)
        .velocityConversionFactor(moduleConstants.drivePCF / 60.0d);

    driveConfig.closedLoop.pid(
        moduleConstants.driveP, moduleConstants.driveI, moduleConstants.driveD);

    driveConfig.smartCurrentLimit(moduleConstants.driveCurrentLimit).idleMode(IdleMode.kBrake);

    return driveConfig;
  }

  private SparkMaxConfig createAngleConfig() {
    SparkMaxConfig angleConfig = new SparkMaxConfig();

    angleConfig.inverted(false);

    angleConfig
        .encoder
        .positionConversionFactor(moduleConstants.anglePCF)
        .velocityConversionFactor(moduleConstants.anglePCF / 60.0d);

    angleConfig
        .closedLoop
        .pid(moduleConstants.angleP, moduleConstants.angleI, moduleConstants.angleD)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-180.0d, 180.0d)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    angleConfig.smartCurrentLimit(moduleConstants.angleCurrentLimit).idleMode(IdleMode.kCoast);

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
    if (isOpenLoop) {
      double motorPercent = state.speedMetersPerSecond / moduleConstants.maxSpeed;
      driveMotor.set(motorPercent);
      driveReference = state.speedMetersPerSecond;
    } else {
      ;
      ffOut = driveFeedforward.calculate(state.speedMetersPerSecond);
      driveController.setReference(
          state.speedMetersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffOut);
      driveReference = state.speedMetersPerSecond;
    }

    if (Constants.sim) {
      driveNeoSim.setInputVoltage(driveSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
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

    if (Constants.sim) {
      angleNeoSim.setInputVoltage(angleSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
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

  public double getAppliedVoltageDrive() {
    return driveSim.getAppliedOutput() * RoboRioSim.getVInVoltage();
  }

  /**
   * SIM ONLY
   *
   * @return
   */
  public double getAngleSimP() {
    return angleMotor.configAccessor.closedLoop.getP();
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
    /* Gets Position from SparkMAX absol encoder * 360 to degrees */
    double positionDeg = absoluteEncoder.getPosition() * 360.0d;

    /* Subtracts magnetic offset to get wheel position */
    positionDeg -= KModuleAbsoluteOffset.getDegrees();

    /* Inverts if necesary */
    positionDeg *= (moduleConstants.absolInverted ? -1 : 1);

    return Rotation2d.fromDegrees(positionDeg);
  }

  public Rotation2d getAbsolutePositionNoOffset() {
    /* Gets Position from SparkMAX absol encoder * 360 to degrees */
    double positionDeg = absoluteEncoder.getPosition() * 360.0d;

    /* Inverts if necesary */
    positionDeg *= (moduleConstants.absolInverted ? -1 : 1);

    return Rotation2d.fromDegrees(positionDeg);
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

  public void updateSimMotors() {
    angleNeoSim.update(0.02);
    driveNeoSim.update(0.02);

    angleSim.iterate(angleNeoSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);
    driveSim.iterate(driveNeoSim.getAngularVelocityRPM() / 60.0d, RoboRioSim.getVInVoltage(), 0.02);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            angleNeoSim.getCurrentDrawAmps(), driveNeoSim.getCurrentDrawAmps()));
  }

  /** Resets the Angle Motor to the position of the absolute position */
  public void setIntegratedAngleToAbsolute() {
    angleEncoder.setPosition(getAbsolutePosition().getDegrees());
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

  public void setNewControlConstants(double[] drive, double[] angle) {
    updateDriveConstants(drive);
    updateAngleConstants(angle);

    driveFeedforward = new SimpleMotorFeedforward(drive[3], drive[4], drive[5]);
  }

  public void updateDriveConstants(double[] drive){
    ClosedLoopConfigAccessor config = driveMotor.configAccessor.closedLoop;

    double p = config.getP();
    double i = config.getI();
    double d = config.getD();

    if(drive[0] != p || drive[1] != i || drive[2] != d){

      SparkMaxConfig newConfig = new SparkMaxConfig();
      newConfig.closedLoop.pid(drive[0], drive[1], drive[2]);
      driveMotor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  public void updateAngleConstants(double[] angle){
    ClosedLoopConfigAccessor config = angleMotor.configAccessor.closedLoop;

    double p = config.getP();
    double i = config.getI();
    double d = config.getD();

    if(angle[0] != p || angle[1] != i || angle[2] != d){
      SparkMaxConfig newConfig = new SparkMaxConfig();
      newConfig.closedLoop.pid(angle[0], angle[1], angle[2]);
      driveMotor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  }
}