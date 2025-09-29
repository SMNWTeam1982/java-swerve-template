package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.Optional;

/** Implements a Vision Subsystem for QuestNav */
public class QuestNavSubsystem extends VisionSubsystem {
  public QuestNav quest = new QuestNav();

  @Override
  public void periodic() {
    quest.commandPeriodic();
    super.periodic();
  }

  public String getName() {
    return "quest nav";
  }

  @Override
  public Command zeroHeading() {
    return runOnce(() -> {});
  }

  protected Optional<VisionData> getVisionResult() {
    Optional<PoseFrame> lastEstimatedPose = Optional.empty();
    PoseFrame[] poseFrames = quest.getAllUnreadPoseFrames();

    if (poseFrames.length > 0) {
      lastEstimatedPose = Optional.of(poseFrames[poseFrames.length - 1]);
    }

    if (lastEstimatedPose.isEmpty()) {
      return Optional.empty();
    }

    return Optional.of(
        new VisionData(
            lastEstimatedPose.get().questPose(),
            lastEstimatedPose.get().dataTimestamp(),
            Constants.VisionConstants.QUESTNAV_CAM_VISION_TRUST));
  }

  /**
   * Method to determine if QuestNav is ready
   *
   * @return True if the Quest is ready and tracking, false otherwise
   */
  public boolean isReady() {
    if (quest.isConnected() && quest.isTracking()) {
      return true;
    }
    return false;
  }

  /**
   * Resets the Quest headset position to the specified pose.
   *
   * @param pose The desired pose as as a {@link Pose2d} Object
   */
  public Command resetPose(Pose2d pose) {
    Pose2d questPose = pose.transformBy(VisionConstants.QUESTNAV_CAM_RELATIVE_TO_ROBOT);
    quest.setPose(questPose);
    return runOnce(() -> {});
  }
}
