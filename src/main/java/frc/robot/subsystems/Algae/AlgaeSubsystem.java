package frc.robot.subsystems.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeSubsystem extends SubsystemBase{ 

private final SparkMax leftMotor;
private final SparkMax rightMotor; 
private final RelativeEncoder leftMotorEncoder;
private final RelativeEncoder rightMotorEncoder;  








public static final SparkBaseConfig LEFT_MOTOR_CONFIG = 
  new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);
  
public static final SparkBaseConfig RIGHT_MOTOR_CONGFIG =  
  new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);
 




 public AlgaeSubsystem() {
    leftMotor = new SparkMax(13, SparkMax.MotorType.kBrushless); // initilizes pivot otor 
    leftMotor.configure(LEFT_MOTOR_CONFIG, 
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters);
        
    rightMotor = new SparkMax(14, SparkMax.MotorType.kBrushless); // initilizes intake motor 
       
        rightMotor.configure(RIGHT_MOTOR_CONGFIG, 
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters);

        leftMotorEncoder = leftMotor.getEncoder();
        leftMotorEncoder.setPosition(0);

        rightMotorEncoder = rightMotor.getEncoder();
        rightMotorEncoder.setPosition(0);
        
 };  

 public static class AlgaeMotorConstants{    


    public static final double ALGAE_INTAKE_MAX_SPEED = 0.5;
    public static final double ALGAE_PDP_CHANNEL = 11;
    public static final double ALGAE_IN_CURRENT_THRESHOLD = 25;
 }

 public Command runintakeMotors() {
    return runOnce(
    () -> {
      leftMotor.set(0.2);
      rightMotor.set(-0.2);
    });
}
public Command runejectMotors() {
  return runOnce(
  () -> {
    leftMotor.set(-0.2);
    rightMotor.set(0.2);
  });
}

public Command stopMotors(){
    return runOnce(
    () -> {
      leftMotor.set(0);
      rightMotor.set(0);
    });
}


}




























