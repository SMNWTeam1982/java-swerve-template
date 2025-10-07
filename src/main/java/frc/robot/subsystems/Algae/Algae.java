import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;
import frc.robot.subsystems.vision.QuestNavSubsystem;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

public class Algae extends SubsystemBase{ 

private SparkMax leftMotor;
private SparkMax rightMotor; 
private RelativeEncoder leftMotorEncoder;
private RelativeEncoder rightMotorEncoder;  





}


public final SparkBaseConfig LEFT_MOTOR_CONFIG = 
    new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);
    





 public Algae() {
    leftMotor = new SparkMax(13, SparkMax.MotorType.kBrushless); // initilizes pivot otor 
    leftMotor.configure(LEFT_MOTOR_CONFIG, 
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters);
        
        rightMotor = new SparkMax(14, SparkMax.MotorType.kBrushless); // initilizes intake motor 
        rightMotor.configure(RIGHT_MOTOR_CONFIG, 
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters);

        leftMotorEncoder = leftMotor.getEncoder();
        leftMotorEncoder.setPosition(0);
        rightMotorEncoder = rightMotor.getEncoder();
        rightMotorEncoder.setPosition(0)
        
 };  