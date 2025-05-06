package frc.robot.subsystems.Vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.QuestNav;
import frc.robot.Constants.VisionConstants;

/**
 * Custom Subsystem for QuestNav that implements several important odometry methods.
 */
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
        return questNavInstance.getPose().transformBy(VisionConstants.questRelativeToRobot.inverse());
    }

    /**
     * Resets the Quest headset position to the specified pose.
     * 
     * @param pose The desired pose as as a Pose2d Object
     */
    public void resetPose(Pose2d pose) {
        Pose2d questPose = pose.transformBy(VisionConstants.questRelativeToRobot.inverse());
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

    /**
     * Method to send telemetry for quest pose data to NetworkTables
     * 
     */
    private void logPoseEstimation() {
        Pose2d currentPose = getEstRobotPose();
        double compositePoseData[] =
                {currentPose.getX(), currentPose.getY(), currentPose.getRotation().getRadians()};
        SmartDashboard.putNumber("QuestNav X", compositePoseData[0]);
        SmartDashboard.putNumber("QuestNav Y", compositePoseData[1]);
        SmartDashboard.putNumber("QuestNav Rotation", compositePoseData[2]);
        SmartDashboard.putNumberArray("QuestNav pose Data", compositePoseData);
    }
}
