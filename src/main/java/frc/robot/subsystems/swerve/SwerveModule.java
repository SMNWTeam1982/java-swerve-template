// This is a one to one re-implementation of the current Python Swerve Code
// I still have no experience with Java so this is more for my learning sake
// Kay - April 2, 2025

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

/** 
 * this is NOT its own subsystem, this is only an abstraction for the drive subsystem that manages one module
 */
public class SwerveModule {

  
  public static class SwerveModuleConstants {
    public static final double POSITION_TO_METERS_MULTIPLIER = 0.31927 / 6.75;
    public static final double RPM_TO_MPS_MULTIPLIER = POSITION_TO_METERS_MULTIPLIER / 60;
    /** 
     * PID constants for the turn motor  
     */
    public static final double TURN_PROPORTIONL_GAIN = 0.73;
    public static final double TURN_INTEGRAL_GAIN = 0.0;
    public static final double TURN_DERIVATIVE_GAIN = 0.01;
    /** 
     * Constant speed for the drive motor 
     */
    public static final double DRIVE_STATIC_GAIN = 0.05;
    public static final double DRIVE_VELOCITY_GAIN_SECONDS_PER_METER = 2.87;

    public static final SparkBaseConfig DRIVE_MOTOR_CONFIG =
        new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);
    public static final SparkBaseConfig TURN_MOTOR_CONFIG =
        new SparkMaxConfig().smartCurrentLimit(30).idleMode(SparkBaseConfig.IdleMode.kCoast);
  }
  
  private SparkMax driveMotor;
  private SparkMax turnMotor;
  private CANcoder turnEncoder;
  private RelativeEncoder driveEncoder;

  private PIDController turnPIDController;

  private SimpleMotorFeedforward driveFeedforward;

  /**
   * Constructs an instance of a SwerveModule with a drive motor, turn motor, and turn encoder.
   */
  public SwerveModule(int driveMotorCANID, int turnMotorCANID, int encoderCANID) {
    driveMotor = new SparkMax(driveMotorCANID, MotorType.kBrushless);
    driveMotor.configure(
        SwerveModuleConstants.DRIVE_MOTOR_CONFIG,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    turnMotor = new SparkMax(turnMotorCANID, MotorType.kBrushless);
    turnMotor.configure(
        SwerveModuleConstants.TURN_MOTOR_CONFIG,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    turnEncoder = new CANcoder(encoderCANID);
    driveEncoder = driveMotor.getEncoder();

    turnPIDController =
        new PIDController(
            SwerveModuleConstants.TURN_PROPORTIONL_GAIN,
            SwerveModuleConstants.TURN_INTEGRAL_GAIN,
            SwerveModuleConstants.TURN_DERIVATIVE_GAIN);

    driveFeedforward =
        new SimpleMotorFeedforward(
            SwerveModuleConstants.DRIVE_STATIC_GAIN,
            SwerveModuleConstants.DRIVE_VELOCITY_GAIN_SECONDS_PER_METER);

    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setDesiredState(SwerveModuleState desiredState) {

    Rotation2d encoderRotation =
        Rotation2d.fromRotations(turnEncoder.getPosition().getValueAsDouble());

    desiredState.optimize(getState().angle);

    desiredState.cosineScale(encoderRotation);

    double driveOutput = driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    if (driveOutput > 12.0) {
      driveOutput = 12.0;
    }
    if (driveOutput < -12.0) {
      driveOutput = -12.0;
    }

    if (Math.abs(driveOutput) < 0.001) {
      driveOutput = 0.0;
    }

    double turnOutput =
        turnPIDController.calculate(encoderRotation.getRadians(), desiredState.angle.getRadians());

    if (turnOutput > 1.0) {
      turnOutput = 1.0;
    }
    if (turnOutput < -1.0) {
      turnOutput = -1.0;
    }

    Logger.recordOutput("Turn Value", turnOutput);
    driveMotor.setVoltage(driveOutput);
    turnMotor.set(-turnOutput);
  }

  /** halts both the turn motor and the drive motor */
  public void stop() {
    turnMotor.set(0);
    driveMotor.set(0);
  }

  /**
   * Updates the Turn motor PID
   *
   * @param p Proportional coefficient
   * @param i Integral coefficient
   * @param d Derivate coefficient
   */
  public void updateTurnPID(double p, double i, double d) {
    turnPIDController.setPID(p, i, d);
  }

  /**
   * @return the angle of the wheel and the velocity of the wheel
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity() * SwerveModuleConstants.RPM_TO_MPS_MULTIPLIER,
        Rotation2d.fromRotations(turnEncoder.getPosition().getValueAsDouble()));
  }

  /**
   * @return the angle of the wheel and the distance traveled by the wheel
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition() * SwerveModuleConstants.POSITION_TO_METERS_MULTIPLIER,
        Rotation2d.fromRotations(turnEncoder.getPosition().getValueAsDouble()));
  }
}
