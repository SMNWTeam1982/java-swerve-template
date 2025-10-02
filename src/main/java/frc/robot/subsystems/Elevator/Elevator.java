package frc.robot.subsystems.Elevator;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private final SparkMax leadMotor;
    private final SparkMax followingMotor;
    private final RelativeEncoder leadEncoder;
    private final RelativeEncoder followingEncoder;
    private final PIDController altitudePidController;

    // using the python code base pid values. 
    private final double ALTITUDE_PROPORTIONAL_GAIN = 5;
    private final double ALTITUDE_INTERGRAL_GAIN = 0;
    private final double ALTITUDE_DERIVATIVE_GAIN = 0;

    private final double ELEVATOR_HEIGHT_OFFSET = 0.56256;
    private final double ELEVATOR_MAX_HEIGHT_METERS = 1.81;
    private final double IDLE_TARGET_HEIGHT = 0.6;

    // The current limit is temporary
    private final SparkBaseConfig LEAD_MOTOR_CONFIG =
        new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);
    private final SparkBaseConfig FOLLOWING_MOTOR_CONFIG =
        (new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast)).inverted(true);
    private final double MOTOR_ROTATIONS_TO_ELEVATOR_HEIGHT_METERS_MULTIPLIER = 1.24744 / 110.5728;
    public Elevator() {
        leadMotor = new SparkMax(11, SparkMax.MotorType.kBrushless);
        followingMotor = new SparkMax(12, SparkMax.MotorType.kBrushless);
        followingMotor.configure(
            FOLLOWING_MOTOR_CONFIG,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
        // inverted following motor in the config in the FOLLOWING_MOTOR_CONFIG init.
        LEAD_MOTOR_CONFIG.follow(followingMotor);
        leadMotor.configure(
            LEAD_MOTOR_CONFIG,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
        leadEncoder = leadMotor.getEncoder();
        followingEncoder = followingMotor.getEncoder();
        zeroEncoders();

        altitudePidController = new PIDController(ALTITUDE_PROPORTIONAL_GAIN, ALTITUDE_INTERGRAL_GAIN, ALTITUDE_DERIVATIVE_GAIN);
        altitudePidController.setTolerance(0.01);



    }

    public Command zeroEncoders() {
        return runOnce(() -> {
            leadEncoder.setPosition(0);
            followingEncoder.setPosition(0);
        });
    }

    public double getElevatorHeight() {
        double position = leadEncoder.getPosition();
        return position * MOTOR_ROTATIONS_TO_ELEVATOR_HEIGHT_METERS_MULTIPLIER;
    }

    public Command moveToHeight(Supplier<Double> doubleSupplier) {
        return runEnd(() -> {
            double currentHeight = getElevatorHeight();
            double pidOutput = altitudePidController.calculate(currentHeight, doubleSupplier.get());
            // Clamp the output to be between -1 and 1
            pidOutput = Math.max(-1, Math.min(1, pidOutput));
            leadMotor.set(pidOutput);
        }, () -> {
            stopMotors();
          }).until(() -> altitudePidController.atSetpoint() || getElevatorHeight() >= ELEVATOR_MAX_HEIGHT_METERS);
    }

    public Command stopMotors() {
        return runOnce(() -> {
            leadMotor.set(0);
            followingMotor.set(0);
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    
}
