// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;
import frc.robot.subsystems.vision.QuestNavSubsystem;
import frc.robot.subsystems.vision.VisionData;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Command-Based Drivetrain subsytem for Swerve Drive */
public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(7, 8, 4);
  private final SwerveModule frontRight = new SwerveModule(1, 2, 3);
  private final SwerveModule rearLeft = new SwerveModule(5, 4, 1);
  private final SwerveModule rearRight = new SwerveModule(3, 6, 2);

  private final Pigeon2 gyro = new Pigeon2(0);

  private Field2d teleopField = new Field2d();
  private Field2d autoField = new Field2d();

  private final Supplier<VisionData> visionDataGetter;
  private final BooleanSupplier visionFreshnessGetter;

  private final SwerveDrivePoseEstimator swervePoseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.swerveKinematics,
    gyro.getRotation2d(),
    new SwerveModulePosition[] {
      frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
          },
          new Pose2d());

  private final PIDController headingController =
      new PIDController(
          DriveConstants.HEADING_PROPORTIONAL_GAIN,
          DriveConstants.HEADING_INTEGRAL_GAIN,
          DriveConstants.HEADING_DERIVATIVE_GAIN);

  // Various overload constructor functions for various vision configurations
  // In competetition both are enabled, but for testing, it is useful to have any combination of
  // vision subsystems enabled.
  /** Initialize Drive Subsystem */
  public DriveSubsystem(
    Supplier<VisionData> visionDataGetter,
    BooleanSupplier visionFreshnessGetter
  ) {
    this.visionDataGetter = visionDataGetter;
    this.visionFreshnessGetter = visionFreshnessGetter;
    zeroHeading();
    SmartDashboard.putData("Teleoperated Field", teleopField);
    SmartDashboard.putData("Autonomous Field", autoField);
    SmartDashboard.putData(
        "Swerve Drive",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");
            builder.addDoubleProperty(
                "Front Left Angle", () -> frontLeft.getState().angle.getRadians(), null);
            builder.addDoubleProperty(
                "Front Left Velocity", () -> frontLeft.getState().speedMetersPerSecond, null);

            builder.addDoubleProperty(
                "Front Right Angle", () -> frontRight.getState().angle.getRadians(), null);
            builder.addDoubleProperty(
                "Front Right Velocity", () -> frontRight.getState().speedMetersPerSecond, null);

            builder.addDoubleProperty(
                "Back Left Angle", () -> rearLeft.getState().angle.getRadians(), null);
            builder.addDoubleProperty(
                "Back Left Velocity", () -> rearLeft.getState().speedMetersPerSecond, null);

            builder.addDoubleProperty(
                "Back Right Angle", () -> rearRight.getState().angle.getRadians(), null);
            builder.addDoubleProperty(
                "Back Right Velocity", () -> rearRight.getState().speedMetersPerSecond, null);

            builder.addDoubleProperty("Robot Angle", () -> getHeading().getRadians(), null);
          }
        });
    configurePathPlanner();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    swervePoseEstimator.update(
      getHeading(),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        rearLeft.getPosition(),
        rearRight.getPosition()
      }
    );

    if (visionFreshnessGetter.getAsBoolean()){
      VisionData lastVisionData = visionDataGetter.get();
      swervePoseEstimator.addVisionMeasurement(
        lastVisionData.pose,
        lastVisionData.timestamp,
        lastVisionData.standardDeviations
      );
    }
    
    logRobotPose(getEstimatedPose());
  }

  /** Init method for configuring PathPlanner to improve readability in constructor */
  public void configurePathPlanner() {
    // Log pathplanner poses and trajectories to custom Field2d object for visualization
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          autoField.getObject("Target").setPose(pose);
          Logger.recordOutput("Drive/Auto/TargetPose", pose);
        });
    PathPlannerLogging.setLogActivePathCallback(
        (poseList) -> {
          Pose2d[] poses = poseList.toArray(new Pose2d[poseList.size()]);
          if (poses.length != 0) {
            autoField.getObject("trajectory").setPoses(poses);
            Logger.recordOutput("Drive/Auto/Trajectory", poses);
          }
        });
    AutoBuilder.configure(
        this::getEstimatedPose,
        this::setEstimatedPose,
        this::getRelativeSpeeds,
        (speeds) -> driveWithChassisSpeeds(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
        AutoConstants.PATHPLANNER_CONFIG,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  /**
   * Resets the odometry to the specified pose.
   * does not affect vision
   *
   * @param pose The pose to which to set the odometry.
   */
  public Command setEstimatedPose(Pose2d pose) {
    return runOnce(
      () -> {
        swervePoseEstimator.resetPosition(
          gyro.getRotation2d(),
          new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
          },
          pose
        );
      }
    )
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states as an Array of SwerveModuleStates
   */
  private void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Method to update SwerveModule states based on given ChassisSpeeds
   *
   * @param speeds ChassisSpeeds to drive robot
   */
  public Command driveWithChassisSpeeds(ChassisSpeeds speeds) {
    return run(
      () -> {
        ChassisSpeeds.discretize(speeds, DriveConstants.DRIVE_PERIOD);
        SwerveModuleState[] moduleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.ARTIFICIAL_MAX_MPS);
        setModuleStates(moduleStates);
      }
    );
  }

  /** Zeroes the heading of the pose estimator, vision not affected */
  public Command zeroHeading() {
    return runOnce(
      () -> {
        swervePoseEstimator.resetRotation(new Rotation2d());
      }
    );
  }

  /** Returns the heading from getEstimatedPose() */
  public Rotation2d getHeading() {
    return getEstimatedPose().getRotation();
  }

  public Pose2d getEstimatedPose() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  /**
   * Calculate new robot heading based on given input
   *
   * @param targetHeading {@link Rotation2d} with desired heading
   * @return Calculated heading
   */
  public double calculateHeading(Rotation2d targetHeading) {
    return headingController.calculate(
        getEstimatedPose().getRotation().getRadians(), targetHeading.getRadians());
  }

  /**
   * Gets the current Swerve Module States
   *
   * @return Array of {@link SwerveModuleState} objects with the current states
   */
  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState()
    };
  }

  /**
   * Returns the robot relative speed as ChassisSpeeds
   *
   * @return Robot relative ChassisSpeeds
   */
  public ChassisSpeeds getRelativeSpeeds() {
    return DriveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  private double getLinearVelocity() {
    ChassisSpeeds relativeSpeeds = getRelativeSpeeds();
    return Math.hypot(relativeSpeeds.vxMetersPerSecond, relativeSpeeds.vyMetersPerSecond);
  }

  /** Method to send telemetry for robot pose data to NetworkTables */
  public void logRobotPose(Pose2d estimatedPose) {
    teleopField.setRobotPose(estimatedPose);
    autoField.setRobotPose(estimatedPose);
    Logger.recordOutput("Drive/Estimated Robot X", estimatedPose.getX());
    Logger.recordOutput("Drive/Estimated Robot Y", estimatedPose.getY());
    Logger.recordOutput("Drive/Estimated Rotation", estimatedPose.getRotation().getRadians());
    Logger.recordOutput("Drive/Estimated Robot Pose", estimatedPose);
    Logger.recordOutput("Drive/Robot Heading", getHeading());
    Logger.recordOutput("Drive/Swerve Module States", getModuleStates());
    Logger.recordOutput("Drive/Linear Velocity", getLinearVelocity());
  }
}
