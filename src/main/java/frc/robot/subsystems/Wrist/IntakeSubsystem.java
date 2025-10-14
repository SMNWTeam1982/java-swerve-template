package frc.robot.subsystems.Wrist;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase{

    public static class IntakeConstants{
        public static final double CORAL_INTAKE_SPEED = 0.6;
        public static final double CORAL_EJECT_SPEED = -0.4;
        public static final SparkBaseConfig INTAKE_MOTOR_CONFIG = new SparkMaxConfig()
            .smartCurrentLimit(35)
            .idleMode(SparkBaseConfig.IdleMode.kBrake);
    }

    private final SparkMax intakeMotor = new SparkMax(15, SparkMax.MotorType.kBrushless);
    
    public IntakeSubsystem() {
        intakeMotor.configure(
            IntakeConstants.INTAKE_MOTOR_CONFIG,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    /** starts the motor intaking and then sets it to stop when interupted */
    public Command intake(){
        return startEnd(
            () -> intakeMotor.set(IntakeConstants.CORAL_INTAKE_SPEED),
            () -> intakeMotor.set(0)
        );
    }

    /** starts the motor ejecting and then sets it to stop when interupted */
    public Command eject(){
        return startEnd(
            () -> intakeMotor.set(IntakeConstants.CORAL_EJECT_SPEED),
            () -> intakeMotor.set(0)
        );
    }

    /** sets the intake motor to start intaking */
    public Command setIntaking() {
        return runOnce(
            () -> intakeMotor.set(IntakeConstants.CORAL_INTAKE_SPEED)
        );
        
    }

    /** sets the intake motor to start ejecting */
    public Command setEjecting() {
        return runOnce(
            () -> intakeMotor.set(IntakeConstants.CORAL_EJECT_SPEED)
        );
    }

    /** sets the intake motor to 0  */
    public Command stop() {
        return runOnce(
            () -> intakeMotor.set(0)
        );
    }
}
