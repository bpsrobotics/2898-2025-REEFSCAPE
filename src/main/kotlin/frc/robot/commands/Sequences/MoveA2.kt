package frc.robot.commands.Sequences

import edu.wpi.first.wpilibj2.command.ScheduleCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.commands.elevator.MoveElevator
import frc.robot.commands.wrist.MoveWrist
import frc.robot.subsystems.Elevator

class MoveA2 {
    val A2CommandSequence = SequentialCommandGroup(
        MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
        MoveElevator(Constants.ElevatorConstants.ElevatorState.A2.position)

    )
    override fun initialize() {
        ScheduleCommand(A2CommandSequence)
    }

    override fun isFinished(): Boolean {
        return Elevator.elevEncoder.equals(Constants.ElevatorConstants.ElevatorState.A2.position)
    }
}