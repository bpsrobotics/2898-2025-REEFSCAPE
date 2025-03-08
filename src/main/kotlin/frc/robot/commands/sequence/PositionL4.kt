package frc.robot.commands.sequence

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.ScheduleCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.commands.elevator.MoveElevator
import frc.robot.commands.elevator.MoveElevatorBy
import frc.robot.commands.wrist.MoveWrist
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.Wrist.coralState
import frc.robot.subsystems.Wrist.presetGoal

class PositionL4 : Command() {
    val commandGroup = if (Wrist.isObstructing) {
        SequentialCommandGroup(MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
            MoveElevator(Constants.ElevatorConstants.ElevatorState.L4.position),
            MoveWrist(Constants.PivotConstants.PivotState.VerticalBranch.position)
        )
    } else {
        SequentialCommandGroup(
            MoveElevator(Constants.ElevatorConstants.ElevatorState.L4.position),
            MoveWrist(Constants.PivotConstants.PivotState.VerticalBranch.position)
        )
    }
    override fun initialize() {
        presetGoal = Constants.PivotConstants.PivotState.VerticalBranch
        coralState = true
        commandGroup.schedule()
    }

    override fun isFinished(): Boolean {
        return commandGroup.isFinished
    }

    override fun end(interrupted: Boolean) {
        println("CANCELLED L4")
        commandGroup.cancel()
    }


}