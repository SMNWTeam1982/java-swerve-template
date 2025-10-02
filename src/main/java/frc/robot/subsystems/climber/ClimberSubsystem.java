package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    private final SparkMax climberMotor;
    ClimberSubsystem() {
        climberMotor = new SparkMax(17, MotorType.kBrushless);
    }
    
    public Command moveClimberOut() {
        return runEnd(
            () -> {
                climberMotor.set(0.1);
            },
            () -> {
                climberMotor.set(0.0);
            }
        );
    }

    public Command moveClimberIn() {
        return runEnd(
            () -> {
                climberMotor.set(-0.1);
            },
            () -> {
                climberMotor.set(0.0);
            }
        );
    }
}
   
