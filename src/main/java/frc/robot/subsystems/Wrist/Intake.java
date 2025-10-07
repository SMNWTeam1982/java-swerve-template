package frc.robot.subsystems.Wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase{
    private final SparkMax intakeMotor;
    private final RelativeEncoder intakeMotorEncoder;
    
    public final SparkBaseConfig INTAKE_MOTOR_CONFIG = 
        new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);
    
        public Intake() {
        intakeMotor = new SparkMax(15, SparkMax.MotorType.kBrushless); // initilizes intake motor 
        intakeMotor.configure(INTAKE_MOTOR_CONFIG, 
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters);

        intakeMotorEncoder = intakeMotor.getEncoder();
        intakeMotorEncoder.setPosition(0);
    }

    public static class intakeConstants{
        public static final double CORAL_INTAKE_SPEED = 0.6;
       public static final double CORAL_EJECT_SPEED = -0.4;
       public static final double ALGAE_INTAKE_MAX_SPEED = 0.5;
    }
    /**
     * moves the climber away from the robot and ready to be used to hold on to the cage
     */
    public Command IntakeCoral() {
        return runOnce(
        () -> {
          intakeMotor.set(1);
        });
    }
    /**
     * moves the cliber arm in, twords the robot so it can hold on to the cage
     * or so it can be out of the way
    */
    public Command ExpelCoral() {
        return runOnce(
        () -> {
          intakeMotor.set(-1);
        });
    }

    public Command StopMotor() {
        return runOnce(
        () -> {
          intakeMotor.set(0);
        });
    }
}
