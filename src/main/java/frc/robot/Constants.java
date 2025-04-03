// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final String PHOTONVISION_FRONT_CAMERA_NAME = "limelight-front";
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class SwerveModuleConstants {
    public static final double POSITION_TO_METERS_TRAVELED_MULTIPLIER = 0.31927 / 6.75;
    public static final double RPM_TO_METERS_PER_SECOND_CONVERSION_MULTIPLIER = POSITION_TO_METERS_TRAVELED_MULTIPLIER
        / 60;

    public static final double DRIVE_PROPORTIONAL_GAIN = 0.0;
    public static final double DRIVE_INTEGRAL_GAIN = 0.0;
    public static final double DRIVE_DERIVATIVE_GAIN = 0.0;

    public static final double TURN_PROPORTIONAL_GAIN = 0.73;
    public static final double TURN_INTEGRAL_GAIN = 0.0;
    public static final double TURN_DERIVATIVE_GAIN = 0.01;

    public static final double DRIVE_STATIC_GAIN_VOLTS = 0.05;
    public static final double DRIVE_VELOCITY_GAIN_VOLT_SECONDS_PER_METER = 2.87;

  }

  public static class DriveConstants {
    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 3.8;

    public static final double SPEED_CAP_METERS_PER_SECOND = 2.5;

    public static final double DRIVE_PERIOD = TimedRobot.kDefaultPeriod;
    public static final boolean GYRO_REVERSED = false;

    public static final Translation2d FRONT_LEFT_TRANSLATION = new Translation2d(0.2635, 0.2635);
    public static final Translation2d FRONT_RIGHT_TRANSLATION = new Translation2d(0.2635, 0.2635);
    public static final Translation2d REAR_LEFT_TRANSLATION = new Translation2d(0.2635, 0.2635);
    public static final Translation2d REAR_RIGHT_TRANSLATION = new Translation2d(0.2635, 0.2635);

    public static final Transform3d CAMERA_POSITION_RELATIVE_TO_ROBOT = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(12.0),
            Units.inchesToMeters(0.0),
            Units.inchesToMeters(9.75)),
        new Rotation3d(
            0.0,
            10.0,
            0.0));

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
        FRONT_LEFT_TRANSLATION,
        FRONT_RIGHT_TRANSLATION,
        REAR_LEFT_TRANSLATION,
        REAR_RIGHT_TRANSLATION);
  }
}
