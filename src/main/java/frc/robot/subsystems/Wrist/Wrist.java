package frc.robot.subsystems.Wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;



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
      public static final Rotation2d LEVEL_1_CORAL_WRIST_POSITION = Rotation2d.fromDegrees(0);
      public static final Rotation2d LEVEL_MID_CORAL_WRIST_POSITION = Rotation2d.fromDegrees(-15);
      public static final Rotation2d LEVEL_4_CORAL_WRIST_POSITION = Rotation2d.fromDegrees(0);
      public static final Rotation2d INTAKE_CORAL_WRIST_POSITION = Rotation2d.fromDegrees(35);
      
      // Preset positions for Arm with Coral
      public static final double CORAL_PRESET_L1 = 0;
      public static final double CORAL_PRESET_L2 = 0.13;
      public static final double CORAL_PRESET_L3 = 0.13;
      public static final double CORAL_PRESET_L4 = 0.0;
      public static final double CORAL_PRESET_PRE_L4 = 1.0 / 16.0;
      public static final double CORAL_PRESET_STOWED = 0.125;
      public static final double CORAL_PRESET_OUT = 0;
      public static final double CORAL_PRESET_UP = 0.25; // Pointing directly upwards
      public static final double CORAL_PRESET_DOWN = -0.25;

      public static final double CORAL_PRESET_GROUND_INTAKE = 0;
      public static final double HARDSTOP_HIGH = 0.32;
      public static final double HARDSTOP_LOW = -0.26;
      public static final double POS_TOLERANCE = Units.degreesToRotations(5);
      public static final double PLACEHOLDER_CORAL_WEIGHT_KG = 0.8;
      
      // Constant for gear ratio (the power that one motor gives to gear)
       private static final double ARM_RATIO = (12.0 / 60.0) * (20.0 / 60.0) * (18.0 / 48.0);

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
