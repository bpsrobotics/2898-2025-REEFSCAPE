package frc.robot.commands.Sequences

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ScheduleCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.commands.elevator.MoveElevator
import frc.robot.commands.wrist.MoveWrist
import frc.robot.subsystems.Elevator

class MoveL4: Command() {
    val L4CommandSequence = SequentialCommandGroup(
        MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
        MoveElevator(Constants.ElevatorConstants.ElevatorState.L4.position),
        MoveWrist(Constants.PivotConstants.PivotState.VerticalBranch.position)
    )
    override fun initialize() {
        ScheduleCommand(L4CommandSequence)
    }

    override fun isFinished(): Boolean {
        return Elevator.elevEncoder.equals(Constants.ElevatorConstants.ElevatorState.L4.position)
    }
}