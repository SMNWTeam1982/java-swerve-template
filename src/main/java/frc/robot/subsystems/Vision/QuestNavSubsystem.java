package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.QuestNav;
import org.littletonrobotics.junction.Logger;

/** Custom Subsystem for QuestNav that implements several important odometry methods. */
public class QuestNavSubsystem extends SubsystemBase {
  public QuestNav questNavInstance = new QuestNav();

  @Override
  public void periodic() {
    questNavInstance.cleanupResponses();
    questNavInstance.processHeartbeat();
    logPoseEstimation();
  }

  /**
   * Gets the estimated pose of the robot based on the headset position
   *
   * @return The robot pose as a Pose2d Object
   */
  public Pose2d getEstRobotPose() {
    return questNavInstance
        .getPose()
        .transformBy(VisionConstants.QUESTNAV_CAM_RELATIVE_TO_ROBOT.inverse());
  }

  /**
   * Resets the Quest headset position to the specified pose.
   *
   * @param pose The desired pose as as a Pose2d Object
   */
  public void resetPose(Pose2d pose) {
    Pose2d questPose = pose.transformBy(VisionConstants.QUESTNAV_CAM_RELATIVE_TO_ROBOT.inverse());
    questNavInstance.setPose(questPose);
  }

  /**
   * Returns the Quest Timestamp of the current data
   *
   * @return The timestamp as a timebase double
   */
  public double getQuestNavTimestamp() {
    return Utils.fpgaToCurrentTime(questNavInstance.getTimestamp());
  }

  /**
   * Method to determine if QuestNav is ready
   *
   * @return true if the Quest is connected and tracking, and false otherwise.
   */
  public boolean isQuestReady() {
    if (questNavInstance.getConnected() && questNavInstance.getTrackingStatus()) {
      return true;
    } else {
      return false;
    }
  }

  /** Method to send telemetry for quest pose data to NetworkTables */
  private void logPoseEstimation() {
    Pose2d currentPose = getEstRobotPose();
    Logger.recordOutput("QuestNav X", currentPose.getX());
    Logger.recordOutput("QuestNav Y", currentPose.getY());
    Logger.recordOutput("QuestNav Rotation", currentPose.getRotation().getRadians());
    Logger.recordOutput("QuestNav Estimated Pose", currentPose);
  }
}
