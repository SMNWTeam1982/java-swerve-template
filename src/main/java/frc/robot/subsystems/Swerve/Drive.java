// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Command-Based re-implementation of the Drivetrain Class in Python
public class Drive extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(7, 8, 4);;
  private final SwerveModule frontRight = new SwerveModule(1, 2, 3);
  private final SwerveModule rearLeft = new SwerveModule(5, 4, 1);
  private final SwerveModule rearRight = new SwerveModule(3, 6, 2);

  private final Pigeon2 gyro = new Pigeon2(0);;

  private Field2d field = new Field2d();

  private final SwerveDrivePoseEstimator swervePoseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.SWERVE_DRIVE_KINEMATICS, new Rotation2d(gyro.getYaw().getValue()),
      new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(),
          rearLeft.getPosition(), rearRight.getPosition()},
      new Pose2d(), VecBuilder.fill(0.05, 0.05, 1), VecBuilder.fill(0.5, 0.5, 1));;

  public PhotonCamera photonLimelightFront =
      new PhotonCamera(OperatorConstants.PHOTONVISION_FRONT_CAMERA_NAME);

  PhotonPoseEstimator photonPoseEstimator =
      new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
          PoseStrategy.LOWEST_AMBIGUITY, DriveConstants.CAMERA_POSITION_RELATIVE_TO_ROBOT);

  public Drive() {
    gyro.reset();

    // Configure AutoBuilder last
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(this::getPose, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a
                               // starting pose)
          this::getRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot
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
          ), config, // The robot configuration
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
          }, this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    SmartDashboard.putData("Field", field);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    final List<PhotonPipelineResult> photonLatestResult =
        photonLimelightFront.getAllUnreadResults();

    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    SmartDashboard.putBoolean("Vision Target Locked",
        photonLatestResult.get(photonLatestResult.size() - 1).hasTargets());
    return photonPoseEstimator.update(photonLatestResult.get(photonLatestResult.size() - 1));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    swervePoseEstimator.update(gyro.getRotation2d(),
        new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(),
            rearLeft.getPosition(), rearRight.getPosition()});
    var visionEst = getEstimatedGlobalPose(swervePoseEstimator.getEstimatedPosition());
    visionEst.ifPresent(est -> {
      swervePoseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
    });

    field.setRobotPose(swervePoseEstimator.getEstimatedPosition());
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
    swervePoseEstimator.resetPosition(gyro.getRotation2d(),
        new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(),
            rearLeft.getPosition(), rearRight.getPosition()},
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = DriveConstants.SWERVE_DRIVE_KINEMATICS
        .toSwerveModuleStates(ChassisSpeeds.discretize(fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot), DriveConstants.DRIVE_PERIOD));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
        DriveConstants.SPEED_CAP_METERS_PER_SECOND);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
        DriveConstants.SPEED_CAP_METERS_PER_SECOND);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRotation3d().getAngle() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  /**
   * Returns the robot relative speed as ChassisSpeeds
   * 
   * @return Robot relative ChassisSpeeds
   */
  public ChassisSpeeds getRelativeSpeeds() {
    SwerveModuleState moduleStates[] =
        {frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState()};
    return DriveConstants.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(moduleStates);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  }

  public void logPoseEstimation() {
    Pose2d currentPose = getPose();
    double compositePoseData[] =
        {currentPose.getX(), currentPose.getY(), currentPose.getRotation().getRadians()};
    field.setRobotPose(currentPose);
    SmartDashboard.putNumber("Robot X", compositePoseData[0]);
    SmartDashboard.putNumber("Robot Y", compositePoseData[1]);
    SmartDashboard.putNumber("Robot Rotation", compositePoseData[2]);
    SmartDashboard.putNumberArray("Robot pose Data", compositePoseData);
  }

  public static Command drive(
    Drive driveTrain,
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier,
    DoubleSupplier rotationSupplier,
    BooleanSupplier fieldRelative) {
      return Commands.run(
        () -> {
          driveTrain.drive(
            xSupplier.getAsDouble() * DriveConstants.SPEED_CAP_METERS_PER_SECOND,
            ySupplier.getAsDouble() * DriveConstants.SPEED_CAP_METERS_PER_SECOND,
            rotationSupplier.getAsDouble() * 3,
            fieldRelative.getAsBoolean()
          );
        },
        driveTrain
      );
    }
}
