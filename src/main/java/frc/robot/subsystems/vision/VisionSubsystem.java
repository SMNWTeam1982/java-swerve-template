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
 * all public methods can be safely called multiple times per loop and will always return the same thing in the same loop.
 * vision hardware is called only once per loop
 */
public abstract class VisionSubsystem extends SubsystemBase {

  private VisionData lastVisionData;
  private boolean isDataFresh = false;

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
   * it is NOT safe to call this multiple times, only call it once per periodic loop.
   * if the vision system being used cannot get its position this will return an empty optional
   */
  protected abstract Optional<VisionData> getVisionResult();

  public abstract Command resetPose(Pose2d newPose);

  public abstract Command zeroHeading();

  /** gets the name of the vision subsystem for logging and differentiation */
  public abstract String getName();

  private void logPoseEstimation(Pose2d estimatedPose) {
    Logger.recordOutput(getName() + "X Position", estimatedPose.getX());
    Logger.recordOutput(getName() + "Y Position", estimatedPose.getY());
    Logger.recordOutput(getName() + "Rotation", estimatedPose.getRotation().getRadians());
    Logger.recordOutput(getName() + "Estimated Pose", estimatedPose);
  }
}
