package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Abstract class for the implementation of a generic Vision Subsystem providing several APIs used
 * in pose estimation
 * 
 * another purpose this class serves is to prevent vision results from getting fetched multiple times per loop
 */
public abstract class VisionSubsystem extends SubsystemBase {

  private VisionData lastVisionData;
  private boolean isDataFresh = false;

  private final String name;

  /**
   * Constructs a generic Vision Subsystem object
   *
   * @param name The name of the subsystem for logging
   */
  public VisionSubsystem(String name) {
    this.name = name;
  }

  @Override
  public void periodic() {
    var visionResult = getVisionResult();
    if (visionResult.isPresent()){
      isDataFresh = true;
      lastVisionData = visionResult.get();
      logPoseEstimation(lastVisionData.pose);
    } else {
      isDataFresh = false;
    }
  }

  public VisionData getLastVisionData(){
    return lastVisionData;
  }

  /**
   * it is safe to call this multiple times
   */
  public Pose2d getLastPose(){
    return lastVisionData.pose;
  }

  /**
   * @return timestamp with the same epoch as the FPGA timestamp
   */
  public double getLastTimestamp(){
    return lastVisionData.timestamp;
  };

  public Matrix<N3, N1> getLastDataStandardDeviations(){
    return lastVisionData.standardDeviations;
  }

  /**
   * @return if the vision sample is new this loop
   */
  public boolean isDataFresh(){
    return isDataFresh;
  }

  /**
   * it is NOT safe to call this multiple times, only call it once per periodic loop
   * if the vision system being used cannot get its position this will return None
   * the optional will contain the pose and the timestamp in seconds
   */
  protected abstract Optional<VisionData> getVisionResult();

  public abstract void resetPose(Pose2d newPose);

  private void logPoseEstimation(Pose2d estimatedPose) {
    Logger.recordOutput(name + "X Position", estimatedPose.getX());
    Logger.recordOutput(name + "Y Position", estimatedPose.getY());
    Logger.recordOutput(name + "Rotation", estimatedPose.getRotation().getRadians());
    Logger.recordOutput(name + "Estimated Pose", estimatedPose);
  }
}
