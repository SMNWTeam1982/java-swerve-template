package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.DriveSubsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveFieldRelative extends Command {
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
  public DriveFieldRelative(
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
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();
    double x = xVelocity.getAsDouble();
    double y = yVelocity.getAsDouble();
    // Resets robot heading if needed
    if (zeroHeading.getAsBoolean()) {
      driveTrain.zeroHeading();
    }
    if (currentAlliance.isPresent() && currentAlliance.get() == Alliance.Red) {
      x = x * -1;
      y = y * -1;
    }
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            x, y, angularVelocity.getAsDouble(), driveTrain.getHeading());
    driveTrain.driveWithChassisSpeeds(speeds);
  }
}
