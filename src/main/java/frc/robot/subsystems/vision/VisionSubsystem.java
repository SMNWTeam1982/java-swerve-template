package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Abstract class for the implementation of a generic Vision Subsystem providing several APIs used
 * in pose estimation
 */
public abstract class VisionSubsystem extends SubsystemBase {

  private String SubsystemName;

  /**
   * Constructs a generic Vision Subsystem object
   *
   * @param name The name of the subsystem for logging
   */
  public VisionSubsystem(String name) {
    SubsystemName = name;
  }

  @Override
  public void periodic() {
    logPoseEstimation(getEstimatedPose());
  }

  /**
   * Method that gets the current estimated robot pose based on vision data
   *
   * @return The estimated robot pose as a {@link Pose2d} object
   */
  public abstract Pose2d getEstimatedPose();

  /**
   * Method that determines if the vision subsystem is ready
   *
   * @return True if the subsystem is ready and tracking, false otherwise
   */
  public abstract boolean isReady();

  /**
   * Method providing the timestamp of the current pose estimate
   *
   * @return FPGA timebase formatted timestamp in seconds
   */
  public abstract double getTimestamp();

  /**
   * Method to log the current estimated pose from the subsystem
   *
   * @param estimatedPose The current estimated pose as a {@link Pose2d}
   */
  private void logPoseEstimation(Pose2d estimatedPose) {
    Logger.recordOutput(SubsystemName + "X Position", estimatedPose.getX());
    Logger.recordOutput(SubsystemName + "Y Position", estimatedPose.getY());
    Logger.recordOutput(SubsystemName + "Rotation", estimatedPose.getRotation().getRadians());
    Logger.recordOutput(SubsystemName + "Estimated Pose", estimatedPose);
  }
}
