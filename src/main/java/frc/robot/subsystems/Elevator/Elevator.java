package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.Intake.Intake;

public class Elevator extends SubsystemBase {
    private Intake intakeSubsystem;
    private SparkMax leadMotor;
    private SparkMax followerMotor;

    private RelativeEncoder leadEncoder;
    private RelativeEncoder followerEncoder;

    private PIDController altitudePIDController;

    public ElevatorConstants.ElevatorState activeState;

    private double targetHeight = ElevatorConstants.IDLE_TARGET_HEIGHT_METERS;

    public Elevator(Intake intake) {
        intakeSubsystem = intake;
        leadMotor = new SparkMax(11, SparkLowLevel.MotorType.kBrushless);
        leadMotor.configure(ElevatorConstants.ALTITUDE_MOTOR_CONFIG,
                SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        followerMotor = new SparkMax(12, SparkLowLevel.MotorType.kBrushless);
        followerMotor.configure(ElevatorConstants.ALTITUDE_MOTOR_CONFIG.follow(11, true),
                SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        leadEncoder = leadMotor.getEncoder();
        followerEncoder = followerMotor.getEncoder();

        altitudePIDController = new PIDController(ElevatorConstants.ALTITUDE_PROPORTIONAL_GAIN,
                ElevatorConstants.ALTITUDE_INTEGRAL_GAIN,
                ElevatorConstants.ALTITUDE_DERIVATIVE_GAIN);

        altitudePIDController.setTolerance(0.01);

        zeroEncoders();
        activeState = ElevatorState.IDLE;
        
    }
    /**
     * Zeros elevator lead motor and follower motor encoders
     * 
     * <p> The follower encoder is only zeroed to insure consistent data collection.
     */
    public void zeroEncoders() {
        leadEncoder.setPosition(0);
        followerEncoder.setPosition(0);
    }

    public double getElevatorHeight() {
        var position = leadEncoder.getPosition();
        
        position *= ElevatorConstants.ROTATIONS_TO_HEIGHT_METERS_MULTIPLIER;
        return position + ElevatorConstants.HEIGHT_OFFSET;
    }

    public void moveElevatorRaw(double amount) {
        leadMotor.set(-amount);
    }

    public void moveElevator() {
        var output = altitudePIDController.calculate(getElevatorHeight(), targetHeight);
        
        if (output > 1.0) {
            output = 1.0;
        }
        if (output < 1.0) {
            output = -1.0;
        }
        leadMotor.set(output);
    }

    private void moveWristThenElevator() {
        intakeSubsystem.runWrist();
        if (intakeSubsystem.coralWristController.atSetpoint()) {
            moveElevator();
        }
    }
    public boolean moveElevatorAndWrist() {
        moveElevator();
        if (altitudePIDController.atSetpoint()) {
            intakeSubsystem.runWrist();
        }

        if (altitudePIDController.atSetpoint() && intakeSubsystem.coralWristController.atSetpoint()) {
            return true;
        } else {
            return false;
        }
    }

    public void runStateMachine() {
        moveElevator();
        intakeSubsystem.runWrist();
    }
    public void stopMotors() {
        moveElevatorRaw(0.0);
        intakeSubsystem.runWristRaw(0.0);
        intakeSubsystem.stopMotors();
    }
    private ElevatorState idleState() {
        moveWristThenElevator();
        intakeSubsystem.runIntakeEject(false);
        return ElevatorState.IDLE;
    }
    private ElevatorState coralScoreState() {
        if (moveElevatorAndWrist()) {
            setIdle();
        }
        return ElevatorState.CORAL_SCORE;
    }

    private ElevatorState algaeScoreState() {
        if (moveElevatorAndWrist()) {
            setIdle();
        }
        return ElevatorState.ALGAE_SCORE;
    }

    private ElevatorState coralIntakeState() {
        if (moveElevatorAndWrist()) {
            setIdle();
        }
        return ElevatorState.CORAL_INTAKE;
    }
    private ElevatorState algaeIntakeState() {
        if (moveElevatorAndWrist()) {
            setIdle();
        }
        return ElevatorState.ALGAE_INTAKE;
    }
    private ElevatorState coralScoreAlgaeIntakeState() {
        if (moveElevatorAndWrist()) {
            setIdle();
        }
        return ElevatorState.CORAL_SCORE_ALGAE_INTAKE;
    }

    public void setL1() {
        targetHeight = ElevatorConstants.L1_TARGET_HEIGHT_METERS;
        intakeSubsystem.setL1();
        activeState = coralScoreState();
    }
    public void setL2() {
        targetHeight = ElevatorConstants.L2_TARGET_HEIGHT_METERS;
        intakeSubsystem.setL2();
        activeState = coralScoreState();
    }
    public void setL3() {
        targetHeight = ElevatorConstants.L3_TARGET_HEIGHT_METERS;
        intakeSubsystem.setL3();
        activeState = coralScoreState();
    }
    public void setL4() {
        targetHeight = ElevatorConstants.L4_TARGET_HEIGHT_METERS;
        intakeSubsystem.setL4();
        activeState = coralScoreState();
    }
    public void setLowAlgae() {
        targetHeight = ElevatorConstants.L2_TARGET_HEIGHT_METERS;
        intakeSubsystem.setAlgae();
        activeState = algaeIntakeState();
    }
    public void setHighAlgae() {
        targetHeight = ElevatorConstants.L3_TARGET_HEIGHT_METERS;
        intakeSubsystem.setAlgae();
        activeState = algaeIntakeState();
    }
    public void setCoralScoreAlgaeIntake() {
        targetHeight = ElevatorConstants.L3_TARGET_HEIGHT_METERS;
        intakeSubsystem.setL3();
        activeState = coralScoreAlgaeIntakeState();
    }
    public void setProcessor() {
        targetHeight = ElevatorConstants.PROCESSOR_TARGET_HEIGHT_METERS;
        intakeSubsystem.setProcessor();
        activeState = algaeScoreState();
    }
    public void setStation() {
        targetHeight = ElevatorConstants.INTAKE_TARGET_HEIGHT_METERS;
        intakeSubsystem.setStation();
        activeState = coralIntakeState();
    }

    private void setIdle() {
        targetHeight = ElevatorConstants.IDLE_TARGET_HEIGHT_METERS;
        intakeSubsystem.setIdle();
        activeState = idleState();
    }
}
