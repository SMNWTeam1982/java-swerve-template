// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Command-Based re-implementation of the Drivetrain Class in Python
public class Drive extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(7, 8, 4);;
  private final SwerveModule frontRight = new SwerveModule(1, 2, 3);
  private final SwerveModule rearLeft = new SwerveModule(5, 4, 1);
  private final SwerveModule rearRight = new SwerveModule(3, 6, 2);

  private final Pigeon2 gyro = new Pigeon2(0);;

  private final SwerveDrivePoseEstimator swervePoseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.SWERVE_DRIVE_KINEMATICS,
      new Rotation2d(gyro.getYaw().getValue()),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
      },
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, 1),
      VecBuilder.fill(0.5, 0.5, 1));;

  PhotonCamera photonLimelightFront = new PhotonCamera(OperatorConstants.PHOTONVISION_FRONT_CAMERA_NAME);

  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
      PoseStrategy.LOWEST_AMBIGUITY,
      DriveConstants.CAMERA_POSITION_RELATIVE_TO_ROBOT);

  public Drive() {
    gyro.reset();
    SmartDashboard.putData("Field", new Field2d());
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    final PhotonPipelineResult photonLatestResult = photonLimelightFront.getLatestResult();

    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update(photonLatestResult);
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
    var visionEst = getEstimatedGlobalPose(swervePoseEstimator.getEstimatedPosition());
    visionEst.ifPresent(
        est -> {
          swervePoseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
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
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = DriveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
      ChassisSpeeds.discretize(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d()): new ChassisSpeeds(xSpeed, ySpeed, rot), DriveConstants.DRIVE_PERIOD
      )
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.SPEED_CAP_METERS_PER_SECOND);
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
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.SPEED_CAP_METERS_PER_SECOND);
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
  
}
