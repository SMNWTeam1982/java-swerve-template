// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Set this to Mode.REPLAY for AdvantageKit Replay
  public static final Mode simMode = Mode.REAL;

  /** Constants for general configuration of the robot project */
  public static class OperatorConstants {
    public static final String PROJECT_NAME = "Swerve Template";
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final boolean ENABLE_QUESTNAV = false;
    public static final boolean ENABLE_PHOTONLIB = false;
    public static final boolean REPLAY_LOGS = false;
  }

  /** Constants to configure QuestNav and PhotonLib vision sources */
  public static class VisionConstants {
    public static final String PHOTON_CAMERA_NAME = "limelight-front";

    public static final Matrix<N3, N1> PHOTON_CAM_VISION_TRUST = VecBuilder.fill(0.5, 0.5, 1);

    public static final Transform3d PHOTON_CAM_RELATIVE_TO_ROBOT =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(12.0), Units.inchesToMeters(0.0), Units.inchesToMeters(9.75)),
            new Rotation3d(0.0, 10.0, 0.0));

    public static final Matrix<N3, N1> QUESTNAV_CAM_VISION_TRUST =
        VecBuilder.fill(0.02, 0.02, 0.035);

    public static final Transform2d QUESTNAV_CAM_RELATIVE_TO_ROBOT =
        new Transform2d(new Translation2d(Units.inchesToMeters(12.0), 0), new Rotation2d(0));
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
