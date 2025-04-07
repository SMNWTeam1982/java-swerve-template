// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Set this to Mode.REPLAY for AdvantageKit Replay
  public static final Mode simMode = Mode.REAL;

  public static class OperatorConstants {
    public static final String PROJECT_NAME = "Swerve Template";
    public static final String PHOTONVISION_FRONT_CAMERA_NAME = "limelight-front";
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final boolean IS_RIGHT_ALGAE_MOTOR_ENABLED = false;
    public static final boolean IS_LEFT_ALGAE_MOTOR_ENABLED = true;
  }

  public static class SwerveModuleConstants {
    public static final double POSITION_TO_METERS_TRAVELED_MULTIPLIER = 0.31927 / 6.75;
    public static final double RPM_TO_METERS_PER_SECOND_CONVERSION_MULTIPLIER =
        POSITION_TO_METERS_TRAVELED_MULTIPLIER / 60;

    public static final double DRIVE_PROPORTIONAL_GAIN = 0.0;
    public static final double DRIVE_INTEGRAL_GAIN = 0.0;
    public static final double DRIVE_DERIVATIVE_GAIN = 0.0;

    public static final double TURN_PROPORTIONAL_GAIN = 0.73;
    public static final double TURN_INTEGRAL_GAIN = 0.0;
    public static final double TURN_DERIVATIVE_GAIN = 0.01;

    public static final double DRIVE_STATIC_GAIN_VOLTS = 0.05;
    public static final double DRIVE_VELOCITY_GAIN_VOLT_SECONDS_PER_METER = 2.87;

    public static final SparkBaseConfig DRIVE_MOTOR_CONFIG =
        new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);
    public static final SparkBaseConfig TURN_MOTOR_CONFIG =
        new SparkMaxConfig().smartCurrentLimit(30).idleMode(SparkBaseConfig.IdleMode.kCoast);

  }

  public static class DriveConstants {
    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 3.8;

    public static final double SPEED_CAP_METERS_PER_SECOND = 2.5;

    public static final double DRIVE_PERIOD = TimedRobot.kDefaultPeriod;
    public static final boolean GYRO_REVERSED = false;

    public static final Translation2d FRONT_LEFT_TRANSLATION = new Translation2d(0.2635, 0.2635);
    public static final Translation2d FRONT_RIGHT_TRANSLATION = new Translation2d(0.2635, -0.2635);
    public static final Translation2d REAR_LEFT_TRANSLATION = new Translation2d(-0.2635, 0.2635);
    public static final Translation2d REAR_RIGHT_TRANSLATION = new Translation2d(-0.2635, -0.2635);

    public static final Transform3d CAMERA_POSITION_RELATIVE_TO_ROBOT =
        new Transform3d(new Translation3d(Units.inchesToMeters(12.0), Units.inchesToMeters(0.0),
            Units.inchesToMeters(9.75)), new Rotation3d(0.0, 10.0, 0.0));

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS =
        new SwerveDriveKinematics(FRONT_LEFT_TRANSLATION, FRONT_RIGHT_TRANSLATION,
            REAR_LEFT_TRANSLATION, REAR_RIGHT_TRANSLATION);
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class ElevatorConstants {
    public static final double L1_TARGET_HEIGHT_METERS = 0.61;
    public static final double L2_TARGET_HEIGHT_METERS = 0.9;
    public static final double L3_TARGET_HEIGHT_METERS = 1.3;
    public static final double L4_TARGET_HEIGHT_METERS = 1.8;
    public static final double INTAKE_TARGET_HEIGHT_METERS = 0.78;

    public static final double HIGH_ALGAE_TARGET_HEIGHT_METERS = 1.2;
    public static final double PROCESSOR_TARGET_HEIGHT_METERS = 0.6;

    public static final double IDLE_TARGET_HEIGHT_METERS = 0.6;

    public static final double ALTITUDE_PROPORTIONAL_GAIN = 0.5;
    public static final double ALTITUDE_INTEGRAL_GAIN = 0.0;
    public static final double ALTITUDE_DERIVATIVE_GAIN = 0.0;

    public static final double ROTATIONS_TO_HEIGHT_METERS_MULTIPLIER = 1.24744 / 110.5728;
    public static final double HEIGHT_OFFSET = 0.56256;
    public static final double MAX_HEIGHT_METERS = 1.81;
    public static final double MIN_HEIGHT_METERS = HEIGHT_OFFSET;

    public static final SparkBaseConfig ALTITUDE_MOTOR_CONFIG =
        new SparkMaxConfig().smartCurrentLimit(30).idleMode(SparkBaseConfig.IdleMode.kCoast);

    public static enum ElevatorState {
      IDLE,
      CORAL_SCORE,
      CORAL_INTAKE,
      ALGAE_SCORE,
      ALGAE_INTAKE,
      CORAL_SCORE_ALGAE_INTAKE,
    }
  }

  public static class IntakeConstants {
    public static final double REEF_ACTIVE_TIME = 0.5;
    public static final double STATION_ACTIVE_TIME = 8.0;

    public static final double CORAL_INTAKE_SPEED = 0.6;
    public static final double CORAL_EJECT_SPEED = -0.4;
    public static final double ALGAE_INTAKE_MAX_SPEED = 0.5;

    public static final Rotation2d L1_CORAL_WRIST_POSITION = Rotation2d.fromDegrees(0);
    public static final Rotation2d MID_LEVEL_CORAL_WRIST_POSTITION = Rotation2d.fromDegrees(-15);
    public static final Rotation2d L4_CORAL_WRIST_POSITION = Rotation2d.fromDegrees(0);
    public static final Rotation2d INTAKE_CORAL_WRIST_POSITION = Rotation2d.fromDegrees(35);

    public static final Rotation2d CORAL_WRIST_STARTING_POSITION = Rotation2d.fromDegrees(72);
    public static final Rotation2d CORAL_WRIST_STOW_POSITION = Rotation2d.fromDegrees(60);

    public static final double CORAL_ROTATIONS_TO_DEGREES_MULTIPLIER = 72 / -5.2857;
    public static final double CORAL_POSITION_OFFSET = 72;

    public static final int ALGAE_RIGHT_MOTOR_PDP_CHANNEL = 11;
    public static final int ALGAE_LEFT_MOTOR_PDP_CHANNEL = 15;
    public static final int CORAL_WRIST_MOTOR_PDP_CHANNEL = 12;
    public static final int CORAL_INTAKE_MOTOR_PDP_CHANNEL = 13;

    //public static final double ALGAE_INTAKE_CURRENT_THRESHHOLD = 25;
    //public static final double CORAL_INTAKE_CURRENT_THRESHHOLD = 10;
    //public static final double CORAL_EJECT_CURRENT_THRESHHOLD = 2;

    public static final double CORAL_WRIST_STATIC_GAIN = 0.01;
    public static final double CORAL_WRIST_GRAVITY_GAIN = 0.6;
    public static final double CORAL_WRIST_VELOCITY_GAIN = 0.0;

    public static final double CORAL_WRIST_PROPORTIONAL_GAIN = 8;
    public static final double CORAL_WRIST_INGEGRAL_GAIN = 0.1;
    public static final double CORAL_WRIST_DERIVATIVE_GAIN = 0.2;

    public static final double CORAL_WRIST_MAX_VELOCITY_RADIANS_PER_SECOND = Math.PI / 4;
    public static final double CORAL_WRIST_MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;

    public static final Constraints CORAL_WRIST_CONSTRAINTS =
        new Constraints(CORAL_WRIST_MAX_VELOCITY_RADIANS_PER_SECOND,
            CORAL_WRIST_MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

    public static final SparkBaseConfig WRIST_MOTOR_CONFIG =
        new SparkMaxConfig().smartCurrentLimit(27);
    public static final SparkBaseConfig INTAKE_MOTOR_CONFIG =
        new SparkMaxConfig().smartCurrentLimit(25);
  }

  public static enum IntakeState {
    /** State for coral and algae intake running in */
    IN,
    /** State for coral and algae intake running out */
    OUT,
    /** State for coral and algae intake not running */
    HOLD
  }
}
