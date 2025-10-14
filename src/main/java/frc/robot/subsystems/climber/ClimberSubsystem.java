package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    public static class ClimberConstants{
        public static final double EXTEND_SPEED = 0.1;
        public static final double RETRACT_SPEED = -0.1;
    }
    private final SparkMax climberMotor;
    public ClimberSubsystem() {
        climberMotor = new SparkMax(17, MotorType.kBrushless);
    }
    /**
     * moves the climber away from the robot and ready to be used to hold on to the cage
     */
    public Command moveClimberOut() {
        return startEnd(
            () -> {
                climberMotor.set(ClimberConstants.EXTEND_SPEED);
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
        return startEnd(
            () -> {
                climberMotor.set(ClimberConstants.RETRACT_SPEED);
            },
            () -> {
                climberMotor.set(0.0);
            }
        );
    }
}
   
