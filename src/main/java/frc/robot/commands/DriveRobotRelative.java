package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.DriveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveRobotRelative extends Command {
  private final DriveSubsystem driveTrain;
  private final DoubleSupplier xVelocity;
  private final DoubleSupplier yVelocity;
  private final DoubleSupplier angularVelocity;
  private final BooleanSupplier zeroHeading;

  /**
   * @param driveSubsystem Existing Drive Subsystem object
   * @param xSupplier Supplier for X Velocity
   * @param ySupplier Supplier for Y Velocity
   * @param angularSupplier Supplier for Angular Velocity
   * @param resetRotation If desired, zero the robot heading and create a new pose
   */
  public DriveRobotRelative(
      DriveSubsystem driveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier angularSupplier,
      BooleanSupplier resetRotation) {
    driveTrain = driveSubsystem;
    xVelocity = xSupplier;
    yVelocity = ySupplier;
    angularVelocity = angularSupplier;
    zeroHeading = resetRotation;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    // Resets robot heading if needed
    if (zeroHeading.getAsBoolean()) {
      driveTrain.zeroHeading();
    }
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            xVelocity.getAsDouble(), yVelocity.getAsDouble(), angularVelocity.getAsDouble());
    driveTrain.driveWithChassisSpeeds(speeds);
  }
}
