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

class PositionA1 : Command() {
//    val commandGroup = SequentialCommandGroup(
//        MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
//        ParallelCommandGroup(
//            MoveElevator(Constants.ElevatorConstants.ElevatorState.A1.position),
//            MoveWrist(Constants.PivotConstants.PivotState.Algae.position)
//        )
    val commandGroup = if (Wrist.isObstructing) {
        SequentialCommandGroup(MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
        MoveElevator(Constants.ElevatorConstants.ElevatorState.A1.position),
            MoveWrist(Constants.PivotConstants.PivotState.Algae.position)

        )
    } else {
    SequentialCommandGroup(
        MoveElevator(Constants.ElevatorConstants.ElevatorState.A1.position),
        MoveWrist(Constants.PivotConstants.PivotState.Algae.position)
    )        }

    override fun initialize() {
        presetGoal = Constants.PivotConstants.PivotState.Algae
        coralState = false
        commandGroup.schedule()
    }

    override fun isFinished(): Boolean {
        return commandGroup.isFinished
    }
    override fun end(interrupted: Boolean) {
        println("CANCELLED A1")
        commandGroup.cancel()
    }
}