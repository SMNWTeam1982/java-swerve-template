package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Abstract class for the implementation of a generic Vision Subsystem providing several APIs used
 * in pose estimation.
 *
 * <p>all public methods can be safely called multiple times per loop and will always return the
 * same thing in the same loop. vision hardware is called only once per loop.
 *
 * <p>if the vision hardware does not have any new data it will return the same thing as last loop,
 * but it will set the data freshness to false
 */
public abstract class VisionSubsystem extends SubsystemBase {

  private VisionData lastVisionData;
  private boolean isDataFresh = false;

  @Override
  public void periodic() {
    var visionResult = getVisionResult();
    if (visionResult.isPresent()) {
      isDataFresh = true;
      lastVisionData = visionResult.get();
      logPoseEstimation(lastVisionData.pose);
    } else {
      isDataFresh = false;
    }
  }

  /** the data from vision combined in a convinient package */
  public VisionData getLastVisionData() {
    return lastVisionData;
  }

  /** the vision pose */
  public Pose2d getLastPose() {
    return lastVisionData.pose;
  }

  /**
   * @return timestamp with the same epoch as the FPGA timestamp, in seconds
   */
  public double getLastTimestamp() {
    return lastVisionData.timestamp;
  }

  /**
   * these are essentially the approximate error in the measurements for the given field position
   *
   * @return (x error meters, y error meters, heading error radians)
   */
  public Matrix<N3, N1> getLastDataStandardDeviations() {
    return lastVisionData.standardDeviations;
  }

  /**
   * @return if the vision subsystem got new data from the hardware this loop
   */
  public boolean isDataFresh() {
    return isDataFresh;
  }

  /**
   * this is the function that gets the results from the hardware, the hardware may not have any results so return type is an optional
   * <p>it is NOT safe to call this multiple times per loop, only call it once per periodic loop. if
   * the vision system being used cannot get its position this will return an empty optional
   */
  protected abstract Optional<VisionData> getVisionResult();

  /** this would mainly be used for questnav, this does nothing if using photonvision */
  public abstract Command resetPose(Pose2d newPose);

  /**
   * the quest has an internal gyroscope and this will reset it, does nothing if using photonvision
   */
  public abstract Command zeroHeading();

  /**
   * gets the name of the vision subsystem for logging, this lets us name each type of vision
   * subsystem
   */
  public abstract String getName();

  /** this logs the x, y, and heading of the vision individualy and as a Pose2d */
  private void logPoseEstimation(Pose2d estimatedPose) {
    Logger.recordOutput(getName() + "X Position", estimatedPose.getX());
    Logger.recordOutput(getName() + "Y Position", estimatedPose.getY());
    Logger.recordOutput(getName() + "Rotation", estimatedPose.getRotation().getRadians());
    Logger.recordOutput(getName() + "Estimated Pose", estimatedPose);
  }
}
