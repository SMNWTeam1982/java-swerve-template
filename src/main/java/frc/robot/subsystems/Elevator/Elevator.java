package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private final SparkMax leadMotor;
    private SparkMax followingMotor;
    private final RelativeEncoder leadEncoder;
    private final RelativeEncoder followingEncoder;
    // The current limit is temporary
    public final SparkBaseConfig LEAD_MOTOR_CONFIG =
        new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);
        public final SparkBaseConfig FOLLOWING_MOTOR_CONFIG =
        (new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast)).inverted(true);

    public Elevator() {
        leadMotor = new SparkMax(10, SparkMax.MotorType.kBrushless);
        followingMotor = new SparkMax(11, SparkMax.MotorType.kBrushless);
        followingMotor.configure(
            FOLLOWING_MOTOR_CONFIG,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
        // inverted following motor in the config in the FOLLOWING_MOTOR_CONFIG init.
        LEAD_MOTOR_CONFIG.follow(followingMotor);
        leadMotor.configure(
            LEAD_MOTOR_CONFIG,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
        leadEncoder = leadMotor.getEncoder();
        followingEncoder = followingMotor.getEncoder();
        zeroEncoders();
        
    }


    public void zeroEncoders() {
        leadEncoder.setPosition(0);
        followingEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    
}
