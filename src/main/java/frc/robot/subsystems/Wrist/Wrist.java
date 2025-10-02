package frc.robot.subsystems.Wrist;

import edu.wpi.first.math.proto.Wpimath;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Wrist extends SubsystemBase{
    private final SparkMax pivotMotor; // Changes the angle of the wrist 
    private final SparkMax intakeMotor; 
    private final RelativeEncoder pivotMotorEncoder; // Encoder for pivotMotor
    private final RelativeEncoder intakeMotorEncoder;

    public final SparkBaseConfig PIVOT_MOTOR_CONFIG = 
    new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);

    public Wrist() {
        pivotMotor = new SparkMax(30, SparkMax.MotorType.kBrushless); // initilizes pivot otor 
        pivotMotor.configure(PIVOT_MOTOR_CONFIG, 
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters);
        
        intakeMotor = new SparkMax(15, SparkMax.MotorType.kBrushless); // initilizes intake motor 
        intakeMotor.configure(PIVOT_MOTOR_CONFIG, 
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters);

        pivotMotorEncoder = pivotMotor.getEncoder();
        pivotMotorEncoder.setPosition(0);
        intakeMotorEncoder = intakeMotor.getEncoder();
        intakeMotorEncoder.setPosition(0);
    }    

    public static class WristMotorConstants{
      public static final LEVEL_1_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(0);
      public static final LEVEL_MID_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(-15);
      public static final LEVEL_4_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(0);
      public static final INTAKE_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(35);
    }
    public Command runMotors() {
        return runOnce(
        () -> {
          pivotMotor.set(1);
          intakeMotor.set(1);
        });
    }

    public Command stopMotors(){
        return runOnce(
        () -> {
          pivotMotor.set(0);
          intakeMotor.set(0);
        });
    }
}
