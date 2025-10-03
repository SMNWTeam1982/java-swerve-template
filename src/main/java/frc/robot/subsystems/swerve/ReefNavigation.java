package frc.robot.subsystems.swerve;

import java.util.Arrays;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class ReefNavigation {
    /** these currently only contain poses for the robot in front of the reef, not poses for positioning on the branches */
    public static final Pose2d[] REEF_SCORING_POSITIONS = new Pose2d[] {
        getScoringPose(17),
        getScoringPose(18),
        getScoringPose(19), // blue reef
        getScoringPose(20),
        getScoringPose(21),
        getScoringPose(22),

        getScoringPose(6),
        getScoringPose(7),
        getScoringPose(8), // red reef
        getScoringPose(9),
        getScoringPose(10),
        getScoringPose(11),
    };

    public static Pose2d getTagPose(int tagID){
        return AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(tagID).get().toPose2d();
    }

    public static Pose2d getScoringPose(int tagIDOfFace){
        Pose2d tagPose = getTagPose(tagIDOfFace);

        Transform2d tagOffset = new Transform2d(
            new Translation2d(
                Units.inchesToMeters(20),
                tagPose.getRotation()
            ),
            new Rotation2d()
        ); // shift the pose 20 inches outward from the face of the tag

        return tagPose.transformBy(tagOffset);
    }

    public static Pose2d getClosestScoringPose(Pose2d robotPose){
        return robotPose.nearest(Arrays.asList(REEF_SCORING_POSITIONS));
    }
}
