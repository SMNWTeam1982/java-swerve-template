package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem.ElevatorConstants;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.swerve.ReefNavigation;

public final class AutonomousCommands {

    public static Command navigateToNearestScoringPose(
        DriveSubsystem drive
    ){
        return drive.moveToPose(
            ReefNavigation.getClosestScoringPose(drive.getEstimatedPose())
        );
    }

    /** this command requires the drive subsystem and the elevator subsystem, meaning their default commands will be cancelled */
    public static Command scoreNearestL2(
        DriveSubsystem drive,
        ElevatorSubsystem elevator
    ){
        return navigateToNearestScoringPose(drive)
            .andThen(elevator.moveToTargetHeight(ElevatorConstants.LEVEL_2_TARGET_HEIGHT));
    }
}
