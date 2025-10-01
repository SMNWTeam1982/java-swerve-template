package frc.robot.subsystems.Wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Wrist extends SubsystemBase{
    private final SparkMax leadMotor;
    private final RealtiveEncoder leadEncoder; 

    public final SparkBaseConfig LEAD_MOTOR_CONFIG = 
    new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);
                                                                // needs channel for motor
    PWMMotorController Wrist_motor1 = new PWMMotorController("Wrist_motor1", channel);

    Wrist_motor1.set(.1);

    Wrist_motor1.stopMotor();
        
}
