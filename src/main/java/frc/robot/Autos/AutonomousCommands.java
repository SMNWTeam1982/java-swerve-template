package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem.ElevatorConstants;
import frc.robot.subsystems.Wrist.IntakeSubsystem;
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
        ElevatorSubsystem elevator,
        IntakeSubsystem intake
    ){
        return navigateToNearestScoringPose(drive)
            .andThen(elevator.moveToTargetHeight(ElevatorConstants.LEVEL_2_TARGET_HEIGHT)) // move the elevator to the target
            .andThen(intake.eject().alongWith(elevator.runPID()).withTimeout(1)) // eject the coral while holding the elevator in place
            .andThen(elevator.setIdle()); // set the elevator to its idle state, and then the command ends and the subsystems resume their default behavior
    }
}
