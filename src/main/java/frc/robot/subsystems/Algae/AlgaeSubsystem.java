package frc.robot.subsystems.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorSubsystem.ElevatorConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeSubsystem extends SubsystemBase{ 

private final SparkMax leftMotor;
private final SparkMax rightMotor; 

public static class AlgaeMotorConstants{    
  public static final double ALGAE_INTAKE_MAX_SPEED = 0.5;
  public static final double ALGAE_PDP_CHANNEL = 11;
  public static final double ALGAE_IN_CURRENT_THRESHOLD = 25;

  public static final SparkBaseConfig LEFT_MOTOR_CONFIG = 
  new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);
  
public static final SparkBaseConfig RIGHT_MOTOR_CONGFIG =  
  new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);
 

}
private final SparkMax rightleadMotor = new SparkMax(14, SparkMax.MotorType.kBrushless);// initilizes lead moter motor 
    private final RelativeEncoder rightleadEncoder = rightleadMotor.getEncoder();

    private final SparkMax leftfollowingMotor = new SparkMax(12, SparkMax.MotorType.kBrushless);
    private final RelativeEncoder leftfollowingEncoder = leftfollowingMotor.getEncoder();






  public AlgaeSubsystem() {
    rightleadMotor.configure(
        AlgaeMotorConstants.LEAD_MOTOR_CONFIG,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
    );
  
    
   
    leftfollowingMotor.configure(
      AlgaeSubsystem.LEFT_FOLLOWING_MOTOR_CONFIG.follow(14,true),
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters);
        
    rightMotor = new SparkMax(14, SparkMax.MotorType.kBrushless); 
       AlgaeMotorConstants.LEAD_MOTOR_CONFIG.follow(14,true);
       
       rightMotor.configure(
        AlgaeSubsystem.RIGHT_LEAD_MOTOR_CONGFIG.lead(14,true),
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters);

        
        
 };  



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




























