package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;
import gg.questnav.questnav.QuestNav;

/** Implements a Vision Subsystem for QuestNav */
public class QuestNavSubsystem extends VisionSubsystem {
  public QuestNav quest = new QuestNav();

  /**
   * Constructs a new QuestNav Subsystem instance
   *
   * @param name The name of the QuestNav instance
   */
  public QuestNavSubsystem(String name) {
    super(name);
  }

  @Override
  public void periodic() {
    quest.commandPeriodic();
    super.periodic();
  }

  /**
   * Gets the estimated pose of the robot based on the headset position
   *
   * @return The estimated robot pose as a {@link Pose2d} object
   */
  public Pose2d getEstimatedPose() {
    return quest.getPose().transformBy(VisionConstants.QUESTNAV_CAM_RELATIVE_TO_ROBOT.inverse());
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
   * Method providing the timestamp of the current pose estimate
   *
   * @return FPGA timebase formatted timestamp in seconds
   */
  public double getTimestamp() {
    return Utils.fpgaToCurrentTime(quest.getDataTimestamp());
  }

  /**
   * Resets the Quest headset position to the specified pose.
   *
   * @param pose The desired pose as as a {@link Pose2d} Object
   */
  public void resetPose(Pose2d pose) {
    Pose2d questPose = pose.transformBy(VisionConstants.QUESTNAV_CAM_RELATIVE_TO_ROBOT);
    quest.setPose(questPose);
  }
}
