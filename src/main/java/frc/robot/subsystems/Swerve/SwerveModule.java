// This is a one to one re-implementation of the current Python Swerve Code
// I still have no experience with Java so this is more for my learning sake
// Kay - April 2, 2025

package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import frc.robot.Constants.SwerveModuleConstants;

/** Individual Swerve Module for a Swerve Drive drivetrain */
public class SwerveModule {
  private SparkMax driveMotor;
  private SparkMax turnMotor;
  private CANcoder turnEncoder;
  private RelativeEncoder driveEncoder;

  private PIDController turnPIDController;

  private SimpleMotorFeedforward driveFeedforward;

  /**
   * Constructs an instance of a SwerveModule with a drive motor, turn motor, and turn encoder.
   *
   * @param driveMotorID CAN ID of the drive motor
   * @param turnMotorID CAN ID of the turning motor
   * @param encoderID CAN ID of the module's absolute encoder
   */
  public SwerveModule(int driveMotorID, int turnMotorID, int encoderID) {
    driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    driveMotor.configure(
        SwerveModuleConstants.DRIVE_MOTOR_CONFIG,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);
    turnMotor.configure(
        SwerveModuleConstants.TURN_MOTOR_CONFIG,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    turnEncoder = new CANcoder(encoderID);
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

  /**
   * Returns the current position of the SwerveModule
   *
   * @return the position of the SwerveModule as a SwerveModulePosition
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition() * SwerveModuleConstants.POSITION_TO_METERS_MULTIPLIER,
        Rotation2d.fromRadians(turnEncoder.getAbsolutePosition().getValue().in(Units.Radians)));
  }

  /**
   * Runs SwerveModule with desired state parameters.
   *
   * @param desiredState Desired state with speed and angle components
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    Rotation2d encoderRotation =
        Rotation2d.fromRadians(turnEncoder.getAbsolutePosition().getValue().in(Units.Radians));

    desiredState.optimize(getState().angle);

    desiredState.cosineScale(encoderRotation);

    double driveOutput = driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    if (driveOutput > 12.0) {
      driveOutput = 12.0;
    }
    if (driveOutput < -12.0) {
      driveOutput = 12.0;
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
   * Returns the current state of the module
   *
   * @return The current SwerveModuleState of the module
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity(),
        Rotation2d.fromRadians(turnEncoder.getAbsolutePosition().getValue().in(Units.Radians)));
  }
  public void stop() {
    turnMotor.set(0);
    driveMotor.set(0);
  }
}