package frc.robot.commands.Sequences

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ScheduleCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.commands.elevator.MoveElevator
import frc.robot.commands.wrist.MoveWrist
import frc.robot.subsystems.Elevator

class MoveA1 : Command() {
    val A1CommandSequence = SequentialCommandGroup(
        MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
        MoveElevator(Constants.ElevatorConstants.ElevatorState.A1.position),
        MoveWrist(Constants.PivotConstants.PivotState.Algae.position)

    )
    override fun initialize() {
        ScheduleCommand(A1CommandSequence)
    }

    override fun isFinished(): Boolean {
        return A1CommandSequence.isFinished
        //fixme return Elevator.elevEncoder.equals(Constants.ElevatorConstants.ElevatorState.A1.position)
    }
}