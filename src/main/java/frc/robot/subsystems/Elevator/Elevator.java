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
    public static class ElevatorConstants {
        public final static double LEVEL_1_TARGET_HEIGHT = 0.61;
        public final static double LEVEL_2_TARGET_HEIGHT = 0.9; //#1.05 # change
        public final static double LEVEL_3_TARGET_HEIGHT = 1.3; //#1.268
        public final static double LEVEL_4_TARGET_HEIGHT = 1.8;

        public final static double ALGAE_2_TARGET_HEIGHT = 1.2;

        public final static double PROCESSOR_TARGET_HEIGHT = 0.6;

        public final static double INTAKING_TARGET_HEIGHT = 0.78;


        // using the python code base pid values. 
        public final static double ALTITUDE_PROPORTIONAL_GAIN = 5;
        public final static double ALTITUDE_INTERGRAL_GAIN = 0;
        public final static double ALTITUDE_DERIVATIVE_GAIN = 0;

        public final static double ELEVATOR_HEIGHT_OFFSET = 0.56256;
        public final static double ELEVATOR_MAX_HEIGHT_METERS = 1.81;
        public final static double IDLE_TARGET_HEIGHT = 0.6;

        // The current limit is temporary
        public final static  SparkBaseConfig LEAD_MOTOR_CONFIG =
            new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);
        public final static double MOTOR_ROTATIONS_TO_ELEVATOR_HEIGHT_METERS_MULTIPLIER = 1.24744 / 110.5728;


    }


    private final SparkMax leadMotor;
    /** this motor will be configured to follow the commands of the lead motor so we dont have to set it */
    private final SparkMax followingMotor;
    private final RelativeEncoder leadEncoder;
    private final PIDController altitudePidController;

    public Elevator() {
        leadMotor = new SparkMax(11, SparkMax.MotorType.kBrushless);
        followingMotor = new SparkMax(12, SparkMax.MotorType.kBrushless);
        leadMotor.configure(
            ElevatorConstants.LEAD_MOTOR_CONFIG,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );

        followingMotor.configure(
            ElevatorConstants.LEAD_MOTOR_CONFIG.follow(11,true),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
        
        leadEncoder = leadMotor.getEncoder();

        zeroEncoders();

        altitudePidController = new PIDController(
            ElevatorConstants.ALTITUDE_PROPORTIONAL_GAIN,
            ElevatorConstants.ALTITUDE_INTERGRAL_GAIN,
            ElevatorConstants.ALTITUDE_DERIVATIVE_GAIN
        );
        altitudePidController.setTolerance(0.01);
    }

    public Command zeroEncoders() {
        return runOnce(() -> {
            leadEncoder.setPosition(0);
        });
    }

    public double getElevatorHeight() {
        return leadEncoder.getPosition() * ElevatorConstants.MOTOR_ROTATIONS_TO_ELEVATOR_HEIGHT_METERS_MULTIPLIER;
    }

    public Command moveToHeight() {
        return run(
            () -> {
                double currentHeight = getElevatorHeight();
                double pidOutput = altitudePidController.calculate(currentHeight);
                // Clamp the output to be between -1 and 1
                pidOutput = Math.max(-1, Math.min(1, pidOutput));
                leadMotor.set(pidOutput);
            }
        );
    }

    public Command setTargetHeight(double targetHeight) {
        return runOnce(
            () -> {
                altitudePidController.setSetpoint(targetHeight);
            }
        );
    }

    public Command stopMotors() {
        return runOnce(
            () -> {
                leadMotor.set(0);
            }
        );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    
}
