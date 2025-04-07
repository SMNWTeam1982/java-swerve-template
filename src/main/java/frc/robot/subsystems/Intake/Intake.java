package frc.robot.subsystems.Intake;

//import java.util.ArrayList;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;

public class Intake extends SubsystemBase {
    private PowerDistribution pdpReference;
    private SparkMax coralWristMotor;
    private SparkMax coralIntakeMotor;
    private SparkMax rightAlgaeIntakeMotor;
    private SparkMax leftAlgaeIntakeMotor;

    private RelativeEncoder coralWristEncoder;

    private ArmFeedforward coralWristFeedForward;

    public ProfiledPIDController coralWristController;

    private Rotation2d coralWristTarget;

//    private ArrayList<Double> previousCoralCurrents;
//    private ArrayList<Double> previousAlgaeCurrents;

    private boolean isIntaking = false;

    public Intake(PowerDistribution powerDistributionModule) {
        pdpReference = powerDistributionModule;
        coralWristMotor = new SparkMax(15, SparkLowLevel.MotorType.kBrushless);
        coralWristMotor.configure(IntakeConstants.WRIST_MOTOR_CONFIG,
                SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        coralWristEncoder = coralWristMotor.getEncoder();

        zeroWristEncoder();

        coralIntakeMotor = new SparkMax(16, SparkLowLevel.MotorType.kBrushless);
        coralIntakeMotor.configure(IntakeConstants.INTAKE_MOTOR_CONFIG,
                SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        if (OperatorConstants.IS_RIGHT_ALGAE_MOTOR_ENABLED) {
            rightAlgaeIntakeMotor = new SparkMax(14, SparkBase.MotorType.kBrushless);
        }
        if (OperatorConstants.IS_LEFT_ALGAE_MOTOR_ENABLED) {
            leftAlgaeIntakeMotor = new SparkMax(13, SparkBase.MotorType.kBrushless);
        }

        coralWristFeedForward = new ArmFeedforward(IntakeConstants.CORAL_WRIST_STATIC_GAIN,
                IntakeConstants.CORAL_WRIST_GRAVITY_GAIN,
                IntakeConstants.CORAL_WRIST_VELOCITY_GAIN);

        coralWristController =
                new ProfiledPIDController(IntakeConstants.CORAL_WRIST_PROPORTIONAL_GAIN,
                        IntakeConstants.CORAL_WRIST_INGEGRAL_GAIN,
                        IntakeConstants.CORAL_WRIST_DERIVATIVE_GAIN,
                        IntakeConstants.CORAL_WRIST_CONSTRAINTS);

        coralWristController.enableContinuousInput(0, 360);
        coralWristController.setTolerance(0.1);
        coralWristController.reset(getWristPosition().getRadians());

        coralWristTarget = IntakeConstants.CORAL_WRIST_STOW_POSITION;

    }

    private void zeroWristEncoder() {
        coralWristEncoder.setPosition(0);
    }

    private Rotation2d getWristPosition() {
        var position = coralWristEncoder.getPosition()
                * IntakeConstants.CORAL_ROTATIONS_TO_DEGREES_MULTIPLIER
                + IntakeConstants.CORAL_POSITION_OFFSET;
        var positionRadians = MathUtil.angleModulus(Units.degreesToRadians(position));
        return new Rotation2d(positionRadians);
    }

    public void stopMotors() {
        coralIntakeMotor.set(0.0);
        if (OperatorConstants.IS_LEFT_ALGAE_MOTOR_ENABLED) {
            leftAlgaeIntakeMotor.set(0.0);
        }
        if (OperatorConstants.IS_RIGHT_ALGAE_MOTOR_ENABLED) {
            rightAlgaeIntakeMotor.set(0.0);
        }
    }

    public void runWristRaw(double amount) {
        coralWristMotor.set(amount);
    }

    public void runWrist() {
        var wristPosition = getWristPosition();
        var pidAmount = coralWristController.calculate(getWristPosition().getRadians(),
                coralWristTarget.getRadians());

        var feedForward = coralWristFeedForward.calculate(wristPosition.getRadians(),
                Units.rotationsPerMinuteToRadiansPerSecond(coralWristEncoder.getVelocity()));

        var output = pidAmount + feedForward;

        if (output > 12.0) {
            output = 12.0;
        }
        if (output < -12.0) {
            output = 12.0;
        }
        coralWristMotor.setVoltage(output);
    }

    public void runIntakeEject(boolean active) {
        if (active) {
            if (!isIntaking) {
                // Intake
                coralIntakeMotor.set(-0.3);
                if (OperatorConstants.IS_LEFT_ALGAE_MOTOR_ENABLED) {
                    leftAlgaeIntakeMotor.set(0.0);
                }
                if (OperatorConstants.IS_RIGHT_ALGAE_MOTOR_ENABLED) {
                    rightAlgaeIntakeMotor.set(0.0);
                }
            } else {
                // Eject
                coralIntakeMotor.set(0.6);
                if (OperatorConstants.IS_LEFT_ALGAE_MOTOR_ENABLED) {
                    leftAlgaeIntakeMotor.set(-0.5);
                }
                if (OperatorConstants.IS_RIGHT_ALGAE_MOTOR_ENABLED) {
                    rightAlgaeIntakeMotor.set(-0.5);
                }
            }
        } else {
            coralIntakeMotor.set(0);
            if (OperatorConstants.IS_LEFT_ALGAE_MOTOR_ENABLED) {
                leftAlgaeIntakeMotor.set(0.0);
            }
            if (OperatorConstants.IS_RIGHT_ALGAE_MOTOR_ENABLED) {
                rightAlgaeIntakeMotor.set(0.0);
            }
        }
    }

    public void logIntakeCurrents() {
        SmartDashboard.putNumber("Coral Intake Current",
                pdpReference.getCurrent(IntakeConstants.CORAL_INTAKE_MOTOR_PDP_CHANNEL));
        if (OperatorConstants.IS_RIGHT_ALGAE_MOTOR_ENABLED) {
            SmartDashboard.putNumber("Algae Intake Current (Right Motor)",
                    pdpReference.getCurrent(IntakeConstants.ALGAE_RIGHT_MOTOR_PDP_CHANNEL));
        }
        if (OperatorConstants.IS_LEFT_ALGAE_MOTOR_ENABLED) {
            SmartDashboard.putNumber("Algae Intake Current (Left Motor)",
                    pdpReference.getCurrent(IntakeConstants.ALGAE_LEFT_MOTOR_PDP_CHANNEL));
        }
    }

    public void logWristPosition() {
        SmartDashboard.putNumber("Calculated Wrist Position (degrees)",
                getWristPosition().getDegrees());
        SmartDashboard.putNumber("Raw Coral Wrist Position (rotations)",
                coralWristEncoder.getPosition());
        SmartDashboard.putNumber("Target Wrist Position (degrees)", coralWristTarget.getDegrees());

    }

    public void logWristSafetyData() {
        SmartDashboard.putNumber("Wrist Current", coralWristMotor.getOutputCurrent());
        SmartDashboard.putNumber("Wrist Temperature", coralWristMotor.getMotorTemperature());
    }

    public void setL1() {
        coralWristTarget = IntakeConstants.L1_CORAL_WRIST_POSITION;
        isIntaking = false;
    }

    public void setL2() {
        coralWristTarget = IntakeConstants.MID_LEVEL_CORAL_WRIST_POSTITION;
        isIntaking = false;
    }

    public void setL3() {
        coralWristTarget = IntakeConstants.MID_LEVEL_CORAL_WRIST_POSTITION;
        isIntaking = false;
    }

    public void setL4() {
        coralWristTarget = IntakeConstants.L4_CORAL_WRIST_POSITION;
        isIntaking = false;
    }

    public void setAlgae() {
        coralWristTarget = IntakeConstants.CORAL_WRIST_STOW_POSITION;
        isIntaking = true;
    }

    public void setProcessor() {
        coralWristTarget = IntakeConstants.CORAL_WRIST_STOW_POSITION;
        isIntaking = false;
    }

    public void setStation() {
        coralWristTarget = IntakeConstants.INTAKE_CORAL_WRIST_POSITION;
        isIntaking = true;
    }

    public void setIdle() {
        coralWristTarget = IntakeConstants.CORAL_WRIST_STOW_POSITION;
        isIntaking = true;
    }
}
