// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Set this to Mode.REPLAY for AdvantageKit Replay
  public static final Mode simMode = Mode.REAL;

  /** Constants for general configuration of the robot project */
  public static class OperatorConstants {
    public static final String projectName = "Swerve Template";
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;
    public static final boolean enableQuestNav = true;
    public static final boolean enablePhotonLib = false;
    public static final boolean isSim = true;
  }

  /** Constants to configure Swerve Modules */
  public static class SwerveModuleConstants {
    public static final double positionToMetersMultiplier = 0.31927 / 6.75;
    public static final double rpmToMpsMultiplier = positionToMetersMultiplier / 60;

    public static final double turnProportionalGain = 0.73;
    public static final double turnIntegralGain = 0.0;
    public static final double turnDerivativeGain = 0.01;

    public static final double driveStaticGain = 0.05;
    public static final double driveVelocityGainSecondsPerMeter = 2.87;

    public static final SparkBaseConfig driveMotorConfig =
        new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);
    public static final SparkBaseConfig turnMotorConfig =
        new SparkMaxConfig().smartCurrentLimit(30).idleMode(SparkBaseConfig.IdleMode.kCoast);
  }

  /** Constants to configure Drivetrain */
  public static class DriveConstants {
    public static final double physicalMaxSpeedMps = 3.8;

    public static final double artificialMaxSpeedMps = 2.5;

    public static final double drivePeriod = TimedRobot.kDefaultPeriod;
    public static final boolean gyroReversed = false;

    public static final double headingPorportionalGain = 1.0;
    public static final double headingIntegralGain = 0.0;
    public static final double headingDerivativeGain = 0.0;

    public static final Translation2d frontLeftTranslation = new Translation2d(0.2635, 0.2635);
    public static final Translation2d frontRightTranslation = new Translation2d(0.2635, -0.2635);
    public static final Translation2d rearLeftTranslation = new Translation2d(-0.2635, 0.2635);
    public static final Translation2d rearRightTranslation = new Translation2d(-0.2635, -0.2635);

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            frontLeftTranslation, frontRightTranslation, rearLeftTranslation, rearRightTranslation);
  }

  /** Constants to configure QuestNav and PhotonLib vision sources */
  public static class VisionConstants {
    public static final String photonFrontCameraName = "limelight-front";

    public static final Matrix<N3, N1> photonLibVisionTrust = VecBuilder.fill(0.5, 0.5, 1);

    public static final Transform3d frontCameraRelativeToRobot =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(12.0), Units.inchesToMeters(0.0), Units.inchesToMeters(9.75)),
            new Rotation3d(0.0, 10.0, 0.0));

    public static final Matrix<N3, N1> questNavVisionTrust = VecBuilder.fill(0.02, 0.02, 0.035);

    public static final Transform2d questRelativeToRobot =
        new Transform2d(new Translation2d(Units.inchesToMeters(12.0), 0), new Rotation2d(0));
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
