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

    /** ^ Initilizes the intake motor ^ */

    public static class intakeConstants{
       public static final double CORAL_INTAKE_SPEED = 0.6;
       public static final double CORAL_EJECT_SPEED = -0.4;
       public static final double ALGAE_INTAKE_MAX_SPEED = 0.5;
    }

    public Command IntakeCoral() {  // Intakes Coral 
        return runOnce(
        () -> {
          intakeMotor.set(1);
        });
    }
    public Command ExpelCoral() { // Expels Coral 
        return runOnce(
        () -> {
          intakeMotor.set(-1);
        });
    }

    public Command StopMotor() { // Stops Intake motor  
        return runOnce(
        () -> {
          intakeMotor.set(0);
        });
    }
}
