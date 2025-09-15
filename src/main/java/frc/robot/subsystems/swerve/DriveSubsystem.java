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
import frc.robot.subsystems.vision.VisionData;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Command-Based Drivetrain subsytem for Swerve Drive */
public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(7, 8, 4);
  private final SwerveModule frontRight = new SwerveModule(1, 2, 3);
  private final SwerveModule backLeft = new SwerveModule(5, 4, 1);
  private final SwerveModule backRight = new SwerveModule(3, 6, 2);

  /**
   * the rotation the gyro represents is not the same as the robot heading the gyro data is used as
   * a reference for the pose estimator the pose estimator offsets the gyro data to get the actual
   * robot heading
   *
   * <p>you do NOT have to zero the gyro
   */
  private final Pigeon2 gyro = new Pigeon2(0);

  private Field2d teleopField = new Field2d();
  private Field2d autoField = new Field2d();

  private final Supplier<VisionData> visionDataGetter;
  private final BooleanSupplier visionFreshnessGetter;

  /** the swerve drive is initialized with a default Pose2d (0 x, 0 y, 0 rotation) */
  private final SwerveDrivePoseEstimator swervePoseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.swerveKinematics,
    gyro.getRotation2d(),
    getModulePositions(),
    new Pose2d()
  );

  /**
   * this controller is used for "top down" robot control. give this controller the current heading
   * and the desired heading and it will output in radians per second
   */
  private final PIDController headingController = new PIDController(
    DriveConstants.HEADING_PROPORTIONAL_GAIN,
    DriveConstants.HEADING_INTEGRAL_GAIN,
    DriveConstants.HEADING_DERIVATIVE_GAIN
  );

  public DriveSubsystem(
      Supplier<VisionData> visionDataGetter, BooleanSupplier visionFreshnessGetter) {
    this.visionDataGetter = visionDataGetter;
    this.visionFreshnessGetter = visionFreshnessGetter;

    SmartDashboard.putData("Teleoperated Field", teleopField);
    SmartDashboard.putData("Autonomous Field", autoField);
    SmartDashboard.putData(
        "Swerve Drive",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");
            builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getState().angle.getRadians(), null);
            builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getState().speedMetersPerSecond, null);
            builder.addDoubleProperty("Front Right Angle", () -> frontRight.getState().angle.getRadians(), null);
            builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getState().speedMetersPerSecond, null);
            builder.addDoubleProperty("Back Left Angle", () -> backLeft.getState().angle.getRadians(), null);
            builder.addDoubleProperty("Back Left Velocity", () -> backLeft.getState().speedMetersPerSecond, null);
            builder.addDoubleProperty("Back Right Angle", () -> backRight.getState().angle.getRadians(), null);
            builder.addDoubleProperty("Back Right Velocity", () -> backRight.getState().speedMetersPerSecond, null);
            builder.addDoubleProperty("Robot Angle", () -> getHeading().getRadians(), null);
          }
        });
    configurePathPlanner();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    swervePoseEstimator.update(gyro.getRotation2d(), getModulePositions());

    if (visionFreshnessGetter.getAsBoolean()) {
      VisionData lastVisionData = visionDataGetter.get();
      swervePoseEstimator.addVisionMeasurement(
          lastVisionData.pose, lastVisionData.timestamp, lastVisionData.standardDeviations);
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
        (pose) -> {},
        this::getRobotRelativeSpeeds,
        (speeds) -> setModulesFromRobotRelativeSpeeds(speeds),
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

  /** Resets the pose estimator to the specified pose. */
  public Command resetEstimatedPose(Pose2d pose, VisionSubsystem vision) {
    return runOnce(
        () -> {
          vision.resetPose(pose);
          swervePoseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
        });
  }



  /** Zeroes the heading of the pose estimator */
  public Command zeroEstimatedHeading(VisionSubsystem vision) {
    return runOnce(
        () -> {
          vision.zeroHeading();
          swervePoseEstimator.resetRotation(new Rotation2d());
        });
  }

  /** sets all of the drivetrain motors to 0 */
  public Command stop() {
    return runOnce(
        () -> {
          frontLeft.stop();
          frontRight.stop();
          backLeft.stop();
          backRight.stop();
        });
  }

  /** drives the robot with chassis speeds relative to the robot coordinate system */
  public Command driveRobotRelative(Supplier<ChassisSpeeds> desiredRobotSpeeds) {
    return run(
        () -> {
          setModulesFromRobotRelativeSpeeds(desiredRobotSpeeds.get());
        });
  }

  /** drives the robot with chassis speeds relative to the field coordinate system */
  public Command driveFieldRelative(Supplier<ChassisSpeeds> desiredFieldSpeeds) {
    return run(
        () -> {
          ChassisSpeeds convertedSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(desiredFieldSpeeds.get(), getHeading());
          setModulesFromRobotRelativeSpeeds(convertedSpeeds);
        });
  }

  /** drives the robot like a top-down shooter
   * <p>the x and y are velocities and the field rotation is relative to the field coordinate system
   */
  public Command driveTopDown(
      DoubleSupplier xVelocityMPS,
      DoubleSupplier yVelocityMPS,
      Supplier<Rotation2d> desiredFieldRotation) {
    return runEnd(
        () -> {
          double pidOutput =
              headingController.calculate(
                  getHeading().getRadians(), desiredFieldRotation.get().getRadians());

          ChassisSpeeds finalSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  xVelocityMPS.getAsDouble(), yVelocityMPS.getAsDouble(), pidOutput, getHeading());

          setModulesFromRobotRelativeSpeeds(finalSpeeds);
        },
        () -> {
          headingController.reset();
        });
  }

  private void setModulesFromRobotRelativeSpeeds(ChassisSpeeds speeds) {
    ChassisSpeeds.discretize(speeds, DriveConstants.DRIVE_PERIOD);
    SwerveModuleState[] moduleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.ARTIFICIAL_MAX_MPS);
    setModuleStates(moduleStates);
  }

  /**
   * calls setDesiredState() on each of the swerve modules
   * @param desiredStates [front left, front right, back left, back right]
   */
  private void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /** Returns the heading from getEstimatedPose() */
  public Rotation2d getHeading() {
    return getEstimatedPose().getRotation();
  }

  /** gets the estimated position from the pose estimator */
  public Pose2d getEstimatedPose() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  /**
   * gets the rotation and velocity of each module
   * @return [front left, front right, back left, back right]
   */
  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()
    };
  }

  /**
   * gets the rotation and distance traveled from each module
   * @return [front left, front right, back left, back right]
   */
  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }

  /**
   * @return robot relative ChassisSpeeds
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  

  /**
   * @return robot relative linear velocity in meters per second
   */
  private double getLinearVelocity() {
    ChassisSpeeds relativeSpeeds = getRobotRelativeSpeeds();
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
    Logger.recordOutput("Drive/Swerve Module States", getModuleStates());
    Logger.recordOutput("Drive/Linear Velocity", getLinearVelocity());
  }
}
