package frc.robot.commands.sequence

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.ScheduleCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.commands.elevator.MoveElevator
import frc.robot.commands.wrist.MoveWrist
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.Wrist.coralState
import frc.robot.subsystems.Wrist.presetGoal

class PositionL3 : Command() {
    val commandGroup = if (Wrist.isObstructing) {
        SequentialCommandGroup(MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
            MoveElevator(Constants.ElevatorConstants.ElevatorState.L3.position),
            MoveWrist(Constants.PivotConstants.PivotState.AngleBranch.position)
        )
    } else {
        SequentialCommandGroup(
        MoveElevator(Constants.ElevatorConstants.ElevatorState.L3.position),
        MoveWrist(Constants.PivotConstants.PivotState.AngleBranch.position)
        )

    }
    override fun initialize() {
        presetGoal = Constants.PivotConstants.PivotState.AngleBranch
        coralState = true
        commandGroup.schedule()
    }

    override fun execute() {
        println("l3")
    }

    override fun isFinished(): Boolean {
        return commandGroup.isFinished
    }
    override fun end(interrupted: Boolean) {
        println("CANCELLED L3")
        commandGroup.cancel()
    }
}