package frc.robot.subsystems.Wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;




public class Pivot extends SubsystemBase{
    private final SparkMax pivotMotor; 
    private final RelativeEncoder pivotMotorEncoder; // Encoder for pivotMotor
    private final PIDController gravityPidController;
    private final PIDController coralWristFeedForeward;

    public final SparkBaseConfig PIVOT_MOTOR_CONFIG = 
    new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);

    public Pivot() {
        pivotMotor = new SparkMax(30, SparkMax.MotorType.kBrushless); // initilizes pivot motor 
        pivotMotor.configure(PIVOT_MOTOR_CONFIG, 
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters);
        pivotMotorEncoder = pivotMotor.getEncoder();
        pivotMotorEncoder.setPosition(0);
       
        coralWristFeedForeward = new PIDController(
          WristMotorConstants.CORAL_WRIST_STATIC_GAIN,
          WristMotorConstants.CORAL_WRIST_GRAVITY_GAIN,
          WristMotorConstants.CORAL_WRIST_VELOCITY_GAIN
        );

        gravityPidController = new PIDController(
        WristMotorConstants.CORAL_WRIST_PROPORTIONAL_GAIN, 
        WristMotorConstants.CORAL_WRIST_DERIVATIVE_GAIN, 
        WristMotorConstants.CORAL_WRIST_INTEGRAL_GAIN
        );
        gravityPidController.setTolerance(0.01);
    }    

    public static class WristMotorConstants{
      public static final double REEF_ACTIVE_TIME = 0.5;
      public static final double STATION_ACTIVE_TIME = 8.0;
      
      public static final Rotation2d LEVEL_1_CORAL_WRIST_POSITION = Rotation2d.fromDegrees(0);
      public static final Rotation2d LEVEL_MID_CORAL_WRIST_POSITION = Rotation2d.fromDegrees(-15);
      public static final Rotation2d LEVEL_4_CORAL_WRIST_POSITION = Rotation2d.fromDegrees(0);
      public static final Rotation2d INTAKE_CORAL_WRIST_POSITION = Rotation2d.fromDegrees(35);
      
      public static final Rotation2d CORAL_WRIST_STARTING_POSITION = Rotation2d.fromDegrees(72);
      public static final Rotation2d CORAL_WRIST_STOW_POSITION = Rotation2d.fromDegrees(60);
  
      public static final double CORAL_ENCODER_ROTATIONS_TO_DEGREES_MULTIPLIER = 72/-5.2857;
      public static final double CORAL_POSITION_OFFSET = 72;

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


       public static final int ALGAE_PDP_CHANNEL = 11;
       public static final int CORAL_PDP_CHANNEL = 13;
       public static final int CORAL_WRIST_PDP_CHANNEL = 12;

       public static final int ALGAE_IN_CURRENT_THRESHOLD = 25;
       public static final int CORAL_IN_CURRENT_THRESHOLD = 10;
       public static final int CORAL_EJECT_CURENT_THRESHOLD = 2;

       public static final double CORAL_WRIST_STATIC_GAIN = 0.01;
       public static final double CORAL_WRIST_GRAVITY_GAIN = 0.6;
       public static final double CORAL_WRIST_VELOCITY_GAIN = 0.0;
    
       // PID values from Python code 
       public static final double CORAL_WRIST_PROPORTIONAL_GAIN = 8;
       public static final double CORAL_WRIST_INTEGRAL_GAIN = 0.1;
       public static final double CORAL_WRIST_DERIVATIVE_GAIN = 0.2;

       public static final double CORAL_WRIST_MAX_VELOCITY_RADIANS_PER_SECOND = Math.PI / 4;
       public static final double CORAL_WRIST_MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;
    }
    public Command getWristPosition(){
        public static position = pivotMotorEncoder.getPosition() * WristMotorConstants.CORAL_ENCODER_ROTATIONS_TO_DEGREES_MULTIPLIER + WristMotorConstants.CORAL_POSITION_OFFSET;
        public static positionRadians = angleModulus(degreesToRadians(position));
        return Rotation2d(positionRadians);
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
