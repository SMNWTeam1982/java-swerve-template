package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/** Custom Subsystem for PhotonVision that implements several important odometry methods */
public class PhotonVisionSubsystem extends SubsystemBase {
  // Define photon cameras here
  private static final PhotonCamera photonLimelightFront =
      new PhotonCamera(VisionConstants.photonFrontCameraName);

  public PhotonPoseEstimator photonPoseEstimator =
      new PhotonPoseEstimator(
          AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
          PoseStrategy.LOWEST_AMBIGUITY,
          VisionConstants.frontCameraRelativeToRobot);

  private PhotonPipelineResult result;

  @Override
  public void periodic() {
    List<PhotonPipelineResult> currentResults = photonLimelightFront.getAllUnreadResults();
    if (!currentResults.isEmpty()) {
      result = currentResults.get(currentResults.size() - 1);
    }
    logPoseEstimation();
  }

  /**
   * Gets the estimated pose of the robot based on recent pipeline results
   *
   * @return The robot pose as a Pose2d Object
   */
  public Pose2d getEstRobotPose() {
    return photonPoseEstimator.update(result).get().estimatedPose.toPose2d();
    // return new Pose2d();
  }

  /**
   * Returns the Camera Timestamp of the current data
   *
   * @return The timestamp as a timebase double
   */
  public double getPhotonVisionTimestamp() {
    return Utils.fpgaToCurrentTime(result.getTimestampSeconds());
  }

  /**
   * Method to determine if Photon is ready
   *
   * @return true if the camera is connected and has targets, and false otherwise.
   */
  public boolean isPhotonReady() {
    if (photonLimelightFront.isConnected() && result.hasTargets()) {
      return true;
    } else {
      return false;
    }
  }

  /** Method to send telemetry for photon pose data to NetworkTables */
  private void logPoseEstimation() {
    Pose2d currentPose = getEstRobotPose();
    double compositePoseData[] = {
      currentPose.getX(), currentPose.getY(), currentPose.getRotation().getRadians()
    };
    SmartDashboard.putNumber("Photon X", compositePoseData[0]);
    SmartDashboard.putNumber("Photon Y", compositePoseData[1]);
    SmartDashboard.putNumber("Photon Rotation", compositePoseData[2]);
    SmartDashboard.putNumberArray("Photon pose Data", compositePoseData);
  }
}
