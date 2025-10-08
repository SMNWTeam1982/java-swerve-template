package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    private final SparkMax climberMotor;
    public ClimberSubsystem() {
        climberMotor = new SparkMax(17, MotorType.kBrushless);
    }
    /**
     * moves the climber away from the robot and ready to be used to hold on to the cage
     */
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
    /**
     * moves the cliber arm in, twords the robot so it can hold on to the cage
     * or so it can be out of the way
    */
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
   
