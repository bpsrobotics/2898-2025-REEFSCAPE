package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants.AutoConstants.RotationD
import frc.robot.Constants.AutoConstants.RotationI
import frc.robot.Constants.AutoConstants.RotationP
import frc.robot.Constants.AutoConstants.TranslationD
import frc.robot.Constants.AutoConstants.TranslationI
import frc.robot.Constants.AutoConstants.TranslationP
import frc.robot.commands.elevator.MoveElevator
import frc.robot.commands.elevator.StabilizeElevator
import frc.robot.commands.intake.RunIntake
import frc.robot.commands.intake.RunOuttake
import frc.robot.commands.sequence.PositionL2
import frc.robot.commands.sequence.Stow
import frc.robot.commands.wrist.MoveWrist
import frc.robot.commands.wrist.StabilizeWrist
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Drivetrain.driveConsumer
import frc.robot.subsystems.Drivetrain.getAlliance
import frc.robot.subsystems.PathPlanner

object Autos {

    init {
        NamedCommands.registerCommand("coralouttake",
            ParallelRaceGroup(
                StabilizeWrist(),
                RunOuttake(0.8, 2.0)
        )) //Todo set this properly
        //todo NamedCommands.registerCommand("algaeintake", AlgaeIntakeOutake())

        NamedCommands.registerCommand("L1", Stow())

        NamedCommands.registerCommand("L2", PositionL2())

        NamedCommands.registerCommand("L3", MoveElevator(Constants.ElevatorConstants.ElevatorState.L3.position))

        NamedCommands.registerCommand("L4", SequentialCommandGroup(
            MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
            ParallelRaceGroup(
                MoveElevator(Constants.ElevatorConstants.ElevatorState.L4.position),
                StabilizeWrist()
            ),
            MoveWrist(Constants.PivotConstants.PivotState.VerticalBranch.position),
            ))

        NamedCommands.registerCommand("PlaceSequence", SequentialCommandGroup(
            MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
            MoveElevator(Constants.ElevatorConstants.ElevatorState.L4.position),
            MoveWrist(Constants.PivotConstants.PivotState.VerticalBranch.position),
            RunOuttake(0.8, 0.5)
        ))

        NamedCommands.registerCommand("autointake", RunIntake())

        NamedCommands.registerCommand("GetCoralStationPiece", SequentialCommandGroup(
            MoveElevator(Constants.ElevatorConstants.ElevatorState.Stow.position),
            MoveWrist(Constants.PivotConstants.PivotState.Stow.position),
            RunIntake()
        ))

        NamedCommands.registerCommand("stabilize", StabilizeElevator())
        PathPlanner
    }


}