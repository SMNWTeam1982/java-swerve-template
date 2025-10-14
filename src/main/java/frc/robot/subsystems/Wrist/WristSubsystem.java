package frc.robot.subsystems.Wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Wrist.IntakeSubsystem.IntakeConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;




public class WristSubsystem extends SubsystemBase{

    public static class WristConstants{
        public static final Rotation2d LEVEL_1_POSITION = Rotation2d.fromDegrees(0);
        public static final Rotation2d LEVEL_MID_POSITION = Rotation2d.fromDegrees(-15);
        public static final Rotation2d LEVEL_4_POSITION = Rotation2d.fromDegrees(0);
        public static final Rotation2d INTAKE_POSITION = Rotation2d.fromDegrees(35);
        public static final Rotation2d STOW_POSITION = Rotation2d.fromDegrees(60);
      
        public static final Rotation2d STARTING_POSITION = Rotation2d.fromDegrees(72); // measured value
        public static final Rotation2d LOWEST_POSITION = Rotation2d.fromDegrees(-30); // this number is yet to be measured
  
        public static final double WRIST_ENCODER_ROTATIONS_TO_DEGREES_MULTIPLIER = 72/-5.2857;
        public static final double WRIST_POSITION_OFFSET = 72;

    //   // Preset positions for Arm with Coral
    //   public static final double CORAL_PRESET_L1 = 0;
    //   public static final double CORAL_PRESET_L2 = 0.13;
    //   public static final double CORAL_PRESET_L3 = 0.13;
    //   public static final double CORAL_PRESET_L4 = 0.0;
    //   public static final double CORAL_PRESET_PRE_L4 = 1.0 / 16.0;
    //   public static final double CORAL_PRESET_STOWED = 0.125;
    //   public static final double CORAL_PRESET_OUT = 0;
    //   public static final double CORAL_PRESET_UP = 0.25; // Pointing directly upwards
    //   public static final double CORAL_PRESET_DOWN = -0.25;

    //   public static final double CORAL_PRESET_GROUND_INTAKE = 0;
    //   public static final double HARDSTOP_HIGH = 0.32;
    //   public static final double HARDSTOP_LOW = -0.26;
    //   public static final double PLACEHOLDER_CORAL_WEIGHT_KG = 0.8;
      
      // Constant for gear ratio (the power that one motor gives to gear)
        private static final double ARM_RATIO = (12.0 / 60.0) * (20.0 / 60.0) * (18.0 / 48.0);

        public static final int WRIST_PDP_CHANNEL = 12;

        public static final double WRIST_STATIC_GAIN = 0.01;
        public static final double WRIST_GRAVITY_GAIN = 0.6;
        public static final double WRIST_VELOCITY_GAIN = 0.0;
    
        // PID values from Python code 
        public static final double WRIST_PROPORTIONAL_GAIN = 8;
        public static final double WRIST_INTEGRAL_GAIN = 0.1;
        public static final double WRIST_DERIVATIVE_GAIN = 0.2;

        public static final double WRIST_PID_TOLERANCE = Units.degreesToRadians(3);

        public static final double WRIST_MAX_VELOCITY_RADIANS_PER_SECOND = Math.PI / 4;
        public static final double WRIST_MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final SparkBaseConfig PIVOT_MOTOR_CONFIG = new SparkMaxConfig()
            .smartCurrentLimit(35)
            .idleMode(SparkBaseConfig.IdleMode.kCoast);

        
    }

    private final SparkMax pivotMotor = new SparkMax(30, SparkMax.MotorType.kBrushless); 

    /** this is in units of rotations */
    private final RelativeEncoder pivotMotorEncoder = pivotMotor.getEncoder(); // Encoder for pivotMotor

    private final ArmFeedforward gravityCompensator = new ArmFeedforward(
        WristConstants.WRIST_STATIC_GAIN,
        WristConstants.WRIST_GRAVITY_GAIN,
        WristConstants.WRIST_VELOCITY_GAIN
    );

    /** position inputs are in radians */
    private final ProfiledPIDController wristController = new ProfiledPIDController(
        WristConstants.WRIST_PROPORTIONAL_GAIN,
        WristConstants.WRIST_INTEGRAL_GAIN,
        WristConstants.WRIST_DERIVATIVE_GAIN,
        new TrapezoidProfile.Constraints(
            WristConstants.WRIST_MAX_VELOCITY_RADIANS_PER_SECOND,
            WristConstants.WRIST_MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED
        )
    );

    

    public WristSubsystem() {
        pivotMotor.configure(
            WristConstants.PIVOT_MOTOR_CONFIG,
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters
        );

        pivotMotorEncoder.setPosition(0);

        wristController.setTolerance(WristConstants.WRIST_PID_TOLERANCE);
        wristController.setGoal(
            WristConstants.STOW_POSITION.getRadians()
        );
    }

    /** the direction of rotation is changed in the encoder config */
    public Rotation2d getWristPosition() {
        double positionDegrees = pivotMotorEncoder.getPosition() * WristConstants.WRIST_ENCODER_ROTATIONS_TO_DEGREES_MULTIPLIER + WristConstants.WRIST_POSITION_OFFSET;
        return Rotation2d.fromDegrees(positionDegrees);
    }

    public Command runPID() {
        return runEnd(
            () -> {
                double wristPositionRadians = getWristPosition().getRadians();

                // a safety mechanism in case the wrist breaks and the encoder goes out of bounds
                if (wristPositionRadians > WristConstants.STARTING_POSITION.getRadians() || wristPositionRadians < WristConstants.LOWEST_POSITION.getRadians()){
                    pivotMotor.set(0);
                    return;
                }

                // pid loop tuned to output in volts
                double pidOutput = wristController.calculate(wristPositionRadians);

                double feedForwardOutput = gravityCompensator.calculate(
                    wristController.getSetpoint().position,
                    wristController.getSetpoint().velocity
                );

                double outputVoltage = MathUtil.clamp(feedForwardOutput + pidOutput, -12, 12);

                pivotMotor.setVoltage(outputVoltage);
            },
            () -> pivotMotor.set(0)
        );
    }

    public Command turnWristUp() { // turns Wrist upwards 
        return runEnd(
        () -> {
          pivotMotor.set(1);
        }, 
        () -> {
          pivotMotor.set(0.0);
      });
    }
    
    public Command turnWristDown(){ // turns Wrist downwards 
      return runEnd(
      () -> {
        pivotMotor.set(-1);
      }, 
      () -> {
        pivotMotor.set(0.0);
    });
  }


  
  public Command runWrist(){
        public static wristPosition = pivotMotorEncoder.getWristPosition();
        public static pidAmount = coralWristFeedForeward.calculate(
          coralWristTarget.getWristPosition().radians(),
          coralWristTarget.radians()
        );

        feedforward = self.coralWristFeedForeward.calculate(
            wristPosition.radians(),
            units.rotationsPerMinuteToRadiansPerSecond(pivotMotorEncoder.getVelocity())
        );

        double output = pidAmount + feedforward;
        
        if (output > 12.0){
            output = 12.0;
        }
        if (output < -12.0){
            output = 12.0;
        }
        
        pivotMotor.setVoltage(output);
  }
}
