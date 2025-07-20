package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/** Implements a Vision Subsystem for Photon Vision */
public class PhotonVisionSubsystem extends VisionSubsystem {

  private PhotonCamera instanceCamera;

  public PhotonPoseEstimator photonPoseEstimator;

  /**
   * Constructs a new Photon Vision Subsystem instance
   *
   * @param name The name of the Photon Vision instance
   */
  public PhotonVisionSubsystem(String name, Transform3d cameraRelativeToRobot, String cameraName) {
    super(name);
    photonPoseEstimator =
        new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
            PoseStrategy.LOWEST_AMBIGUITY,
            cameraRelativeToRobot);
    instanceCamera = new PhotonCamera(cameraName);
  }

  @Override
  public void periodic() {
    updatePoseEstimation(getLatestResult());
    super.periodic();
  }

  /**
   * Method that gets the current estimated robot pose based on vision data
   *
   * @return The estimated robot pose as a {@link Pose2d} object
   */
  public Pose2d getEstimatedPose() {
    return updatePoseEstimation(getLatestResult()).get().estimatedPose.toPose2d();
  }

  /**
   * Method to determine if Photon is ready
   *
   * @return true if the camera is connected and has targets, and false otherwise.
   */
  public boolean isReady() {
    if (instanceCamera.isConnected() && getLatestResult().hasTargets()) {
      return true;
    }
    return false;
  }

  /**
   * Returns the Camera Timestamp of the current data
   *
   * @return The timestamp as a timebase double
   */
  public double getTimestamp() {
    return Utils.fpgaToCurrentTime(getLatestResult().getTimestampSeconds());
  }

  // Photon Vision specific Methods

  /**
   * Method that returns the latest pipeline result if it exists
   *
   * @return PhotonPipelineResult with latest position
   */
  private PhotonPipelineResult getLatestResult() {
    List<PhotonPipelineResult> currentResults = instanceCamera.getAllUnreadResults();
    if (!currentResults.isEmpty()) {
      PhotonPipelineResult result = currentResults.get(currentResults.size() - 1);
      return result;
    } else {
      return new PhotonPipelineResult();
    }
  }

  /**
   * Updates the pose estimation based on vision data
   *
   * @param latestResult The latest {@link PhotonPipelineResult}
   * @return Optional {@link EstimatedRobotPose} based on vision data
   */
  private Optional<EstimatedRobotPose> updatePoseEstimation(PhotonPipelineResult latestResult) {
    return photonPoseEstimator.update(latestResult);
  }
}
