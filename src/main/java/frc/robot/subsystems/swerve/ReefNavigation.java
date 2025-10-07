package frc.robot.subsystems.swerve;

import java.util.Arrays;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class ReefNavigation {
    /** the poses the robot needs to be at in order to score on the reef */
    public static final Pose2d[] REEF_SCORING_POSES = new Pose2d[] {
        getLeftBranchScoringPose(17),
        getRightBranchScoringPose(17),

        getLeftBranchScoringPose(18),
        getRightBranchScoringPose(18),

        getLeftBranchScoringPose(19),
        getRightBranchScoringPose(19),

        getLeftBranchScoringPose(20), // blue reef
        getRightBranchScoringPose(20),

        getLeftBranchScoringPose(21),
        getRightBranchScoringPose(21),

        getLeftBranchScoringPose(22),
        getRightBranchScoringPose(22),



        getLeftBranchScoringPose(6),
        getRightBranchScoringPose(6),

        getLeftBranchScoringPose(7),
        getRightBranchScoringPose(7),

        getLeftBranchScoringPose(8),
        getRightBranchScoringPose(8),

        getLeftBranchScoringPose(9), // red reef
        getRightBranchScoringPose(9),

        getLeftBranchScoringPose(10),
        getRightBranchScoringPose(10),

        getLeftBranchScoringPose(11),
        getRightBranchScoringPose(11),
    };

    /** these vectors are translations from a tag with an angle of 0 (such as tag 21),
     * they are shifted out by 20 inches (to account for the robot width), down 5.5 (for the intake offset), then up or down 6.5 for the left or right branch
     * <p> to use these, rotate them to the angle of the tag then add them to its position to get the translation of the scoring pose
     */

    public static final Translation2d RIGHT_SCORING_VECTOR = new Translation2d(
        Units.inchesToMeters(20),
        Units.inchesToMeters(1)
    );
    public static final Translation2d LEFT_SCORING_VECTOR = new Translation2d(
        Units.inchesToMeters(20),
        Units.inchesToMeters(-12)
    );

    /** puts a Field2d object onto the SmartDashboard that contains the REEF_SCORING_POSES */
    public static void displayScoringPoses(){
        Field2d field = new Field2d();
        field.getObject("scoring poses").setPoses(Arrays.asList(REEF_SCORING_POSES));
        SmartDashboard.putData(field);
    }

    /** gets the field pose of the tag */
    public static Pose2d getTagPose(int tagID){
        return AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(tagID).get().toPose2d();
    }

    public static Pose2d getRightBranchScoringPose(int tagIDOfFace){
        Pose2d tagPose = getTagPose(tagIDOfFace);

        Translation2d rightBranch = RIGHT_SCORING_VECTOR.rotateBy(tagPose.getRotation());

        Pose2d rightBranchTarget = tagPose.transformBy(
            new Transform2d(
                rightBranch,
                new Rotation2d(Math.PI)
            )
        );

        return rightBranchTarget;
    }

    public static Pose2d getLeftBranchScoringPose(int tagIDOfFace){
        Pose2d tagPose = getTagPose(tagIDOfFace);

        Translation2d leftBranch = LEFT_SCORING_VECTOR.rotateBy(tagPose.getRotation());

        Pose2d leftBranchTarget = tagPose.transformBy(
            new Transform2d(
                leftBranch,
                new Rotation2d(Math.PI)
            )
        );

        return leftBranchTarget;
    }

    /**
     * @param robotPose the current pose of the robot on the field
     * @return the closest branch scoring pose to the given robot pose
     */
    public static Pose2d getClosestScoringPose(Pose2d robotPose){
        return robotPose.nearest(Arrays.asList(REEF_SCORING_POSES));
    }
}
