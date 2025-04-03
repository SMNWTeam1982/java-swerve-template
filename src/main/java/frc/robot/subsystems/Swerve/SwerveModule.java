// This is a one to one re-implementation of the current Python Swerve Code
// I still have no experience with Java so this is more for my learning sake
// Kay - April 2, 2025

package frc.robot.subsystems.Swerve;

import frc.robot.Constants.SwerveModuleConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule extends SubsystemBase {
    private SparkMax driveMotor;
    private SparkMax turnMotor;
    private CANcoder turnEncoder;
    private RelativeEncoder driveEncoder;

    private PIDController drivePIDController;
    private PIDController turnPIDController;

    private SimpleMotorFeedforward driveFeedforward;

    /**
     * Constructs an instance of a SwerveModule with a drive motor, turn motor, and
     * turn encoder.
     *
     * @param driveMotorID CAN ID of the drive motor
     * @param turnMotorID  CAN ID of the turning motor
     * @param encoderID    CAN ID of the module's absolute encoder
     */
    public SwerveModule(int driveMotorID, int turnMotorID, int encoderID) {
        this.driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless); // in java you only need to use the this keyword if the parameter name and the property name are the same
        this.turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);

        this.turnEncoder = new CANcoder(encoderID);
        this.driveEncoder = driveMotor.getEncoder();

        this.drivePIDController = new PIDController(
                SwerveModuleConstants.DRIVE_PROPORTIONAL_GAIN,
                SwerveModuleConstants.DRIVE_INTEGRAL_GAIN,
                SwerveModuleConstants.DRIVE_DERIVATIVE_GAIN
        );

        this.turnPIDController = new PIDController(
                SwerveModuleConstants.TURN_PROPORTIONAL_GAIN,
                SwerveModuleConstants.TURN_INTEGRAL_GAIN,
                SwerveModuleConstants.TURN_DERIVATIVE_GAIN
        );

        this.driveFeedforward = new SimpleMotorFeedforward(
                SwerveModuleConstants.DRIVE_STATIC_GAIN_VOLTS,
                SwerveModuleConstants.DRIVE_VELOCITY_GAIN_VOLT_SECONDS_PER_METER
        );

        this.turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current position of the SwerveModule
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition() * SwerveModuleConstants.POSITION_TO_METERS_TRAVELED_MULTIPLIER,
                Rotation2d.fromRotations(turnEncoder.getPosition().getValueAsDouble())
        );
    }

    /**
     * Sets the desired state for the SwerveModule
     * 
     * @param desiredState Desired state with speed (m/s) and angle (Rotation2d)
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d encoderRotation = Rotation2d.fromRotations(turnEncoder.getPosition().getValueAsDouble());

        desiredState.optimize(encoderRotation);

        desiredState.cosineScale(encoderRotation);

        double drivePidOutput = drivePIDController.calculate(
                driveEncoder.getVelocity() * SwerveModuleConstants.RPM_TO_METERS_PER_SECOND_CONVERSION_MULTIPLIER,
                desiredState.speedMetersPerSecond
        );

        double driveOutput = drivePidOutput + driveFeedforward.calculate(desiredState.speedMetersPerSecond);

        if (driveOutput > 12.0) {
            driveOutput = 12.0;
        }
        if (driveOutput < -12.0) {
            driveOutput = 12.0;
        }

        double turnOutput = turnPIDController.calculate(
                encoderRotation.getRadians(),
                desiredState.angle.getRadians()
        );

        if (turnOutput > 1.0) {
            turnOutput = 1.0;
        }
        if (turnOutput < -1.0) {
            turnOutput = -1.0;
        }

        driveMotor.setVoltage(driveOutput);
        turnMotor.setVoltage(-turnOutput);
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
     * Updates the Drive motor PID
     * 
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Derivate coefficient
     */
    public void updateDrivePID(double p, double i, double d) {
        drivePIDController.setPID(p, i, d);
    }

    /**
     * Returns the current state of the module
     * 
     * @return The current SwerveModuleState of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            Rotation2d.fromRotations(turnEncoder.getPosition().getValueAsDouble())
        );
    }
}
