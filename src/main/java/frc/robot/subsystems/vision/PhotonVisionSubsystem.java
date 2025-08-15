package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/** Implements a Vision Subsystem for Photon Vision */
public class PhotonVisionSubsystem extends VisionSubsystem {

  private PhotonCamera instanceCamera;

  public PhotonPoseEstimator photonPoseEstimator;

  public PhotonVisionSubsystem(
    Transform3d cameraRelativeToRobot, 
    String cameraName
  ) {
    photonPoseEstimator = new PhotonPoseEstimator(
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
      PoseStrategy.LOWEST_AMBIGUITY,
      cameraRelativeToRobot
    );
    instanceCamera = new PhotonCamera(cameraName);
  }

  @Override
  public String getName(){
    return "photon vision";
  }

  @Override
  protected Optional<VisionData> getVisionResult(){
    Optional<EstimatedRobotPose> lastEstimatedPose = Optional.empty();

    for (var result : instanceCamera.getAllUnreadResults()){
      lastEstimatedPose = photonPoseEstimator.update(result);
    } // if getAllUnreadResults() is empty then lastEstimatedPose will be Optional.empty()
    // this also accounts for results that have data but are surrounded by results without data

    if (lastEstimatedPose.isEmpty()){
      return Optional.empty();
    }

    EstimatedRobotPose estimatedPose = lastEstimatedPose.get();

    return Optional.of(
      new VisionData(
        estimatedPose.estimatedPose.toPose2d(),
        estimatedPose.timestampSeconds,
        Constants.VisionConstants.PHOTON_CAM_VISION_TRUST // we should calculate this the same way photonVision does in their example code
      )
    );
  }

  @Override
  public Command resetPose(Pose2d newPose){
    return runOnce(() -> {});
  } // this doesn't apply to photonvision

  @Override
  public Command zeroHeading(){
    return runOnce(() -> {});
  }
}
