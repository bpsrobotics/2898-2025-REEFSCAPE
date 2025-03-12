package frc.robot.commands.sequence

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.ScheduleCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.commands.elevator.MoveElevator
import frc.robot.commands.wrist.MoveWrist

class PositionL2 : Command() {
    val commandGroup = SequentialCommandGroup(
        MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
        ParallelCommandGroup(
            MoveElevator(Constants.ElevatorConstants.ElevatorState.L2.position),
            MoveWrist(Constants.PivotConstants.PivotState.AngleBranch.position)
        )
    )
    override fun initialize() {
        commandGroup.schedule()
    }

    override fun execute() {
        println("L2")
    }

    override fun isFinished(): Boolean {
        return commandGroup.isFinished
    }
}