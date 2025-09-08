package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

// i had to make this stupid class because java does not have tuples: (Pose2d, double, Matrix<N3,
// N1>)
public class VisionData {
  public final Pose2d pose;
  public final double timestamp;
  public final Matrix<N3, N1> standardDeviations;

  public VisionData(Pose2d pose, double timestamp, Matrix<N3, N1> standardDeviations) {
    this.pose = pose;
    this.timestamp = timestamp;
    this.standardDeviations = standardDeviations;
  }
}
