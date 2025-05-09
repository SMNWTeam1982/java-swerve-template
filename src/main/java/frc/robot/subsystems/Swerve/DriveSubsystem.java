// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.QuestNavSubsystem;
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

  private Field2d field = new Field2d();

  private QuestNavSubsystem questNav = new QuestNavSubsystem();
  // private PhotonVisionSubsystem photonVision = new PhotonVisionSubsystem();

  private final SwerveDrivePoseEstimator swervePoseEstimator =
      new SwerveDrivePoseEstimator(
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

  public DriveSubsystem() {
    zeroHeading();

    // Configure AutoBuilder last
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a
          // starting pose)
          this::getRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds) -> driveWithChassisSpeeds(speeds), // Method that will drive the robot
          // given ROBOT RELATIVE
          // ChassisSpeeds. Also optionally
          // outputs individual
          // module feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following
              // controller for
              // holonomic
              // drive trains
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
              ),
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
          );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    swervePoseEstimator.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
        });
    // If QuestNav is enabled and ready, add it's measurements
    if (questNav.isQuestReady() && OperatorConstants.ENABLE_QUESTNAV) {
      swervePoseEstimator.addVisionMeasurement(
          questNav.getEstRobotPose(),
          questNav.getQuestNavTimestamp(),
          VisionConstants.QUESTNAV_CAM_VISION_TRUST);
    }
    // If PhotonVision is enabled and ready, add it's measurements
    // if (OperatorConstants.ENABLE_PHOTONLIB) {
    //   swervePoseEstimator.addVisionMeasurement(
    //       photonVision.getEstRobotPose(),
    //       photonVision.getPhotonVisionTimestamp(),
    //       VisionConstants.PHOTON_CAM_VISION_TRUST);
    // }

    field.setRobotPose(getPose());
    logRobotPose();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose as a Pose2d object.
   */
  public Pose2d getPose() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    swervePoseEstimator.resetPosition(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
        },
        pose);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states as an Array of SwerveModuleStates
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.ARTIFICIAL_MAX_MPS);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
    questNav.resetPose(new Pose2d());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading represented as a Rotation2d
   */
  public Rotation2d getHeading() {
    return gyro.getRotation2d();
  }

  /**
   * Returns the robot relative speed as ChassisSpeeds
   *
   * @return Robot relative ChassisSpeeds
   */
  public ChassisSpeeds getRelativeSpeeds() {
    SwerveModuleState moduleStates[] = {
      frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState()
    };
    return DriveConstants.swerveKinematics.toChassisSpeeds(moduleStates);
  }

  /** Method to send telemetry for robot pose data to NetworkTables */
  public void logRobotPose() {
    Pose2d currentPose = getPose();
    SwerveModuleState[] currentStates =
        new SwerveModuleState[] {
          frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState()
        };
    field.setRobotPose(currentPose);
    Logger.recordOutput("Robot X", currentPose.getX());
    Logger.recordOutput("Robot Y", currentPose.getY());
    Logger.recordOutput("Robot Rotation", currentPose.getRotation().getRadians());
    Logger.recordOutput("Robot Pose", currentPose);
    Logger.recordOutput("Swerve Module States", currentStates);
  }

  /**
   * Method to update SwerveModule states based on given ChassisSpeeds
   *
   * @param speeds ChassisSpeeds to drive robot
   */
  public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
    ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] moduleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.ARTIFICIAL_MAX_MPS);
    setModuleStates(moduleStates);
  }

  /**
   * Command to drive the robot relative to the field with heading angle
   *
   * @param x Supplies X-value from controller
   * @param y Supplies Y-value from controller
   * @param targetHeading Supplies target heading from controller
   */
  public Command driveFieldRelativeWithHeading(
      DoubleSupplier x, DoubleSupplier y, Supplier<Rotation2d> targetHeading) {
    return driveFieldRelative(
        x,
        y,
        () ->
            headingController.calculate(
                getPose().getRotation().getRadians(), targetHeading.get().getRadians()));
  }

  /**
   * Command to drive the robot relative to the field
   *
   * @param x Supplies X-value from controller
   * @param y Supplies Y-value from controller
   * @param rot Supplies Angular velocity for rotation from controller
   */
  public Command driveFieldRelative(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    return run(
        () -> {
          ChassisSpeeds speeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  x.getAsDouble(), y.getAsDouble(), rot.getAsDouble(), getPose().getRotation());
          driveWithChassisSpeeds(speeds);
        });
  }

  /**
   * Command to drive the robot relative to it's current position
   *
   * @param x Supplies X-value from controller
   * @param y Supplies Y-value from controller
   * @param rot Supplies Angular velocity for rotation from controller
   */
  public Command driveRobotRelative(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    return run(
        () -> {
          ChassisSpeeds speeds =
              new ChassisSpeeds(x.getAsDouble(), y.getAsDouble(), rot.getAsDouble());
          driveWithChassisSpeeds(speeds);
        });
  }
}
