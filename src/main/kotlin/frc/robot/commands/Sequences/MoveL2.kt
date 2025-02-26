package frc.robot.commands.Sequences

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ScheduleCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.commands.elevator.MoveElevator
import frc.robot.commands.wrist.MoveWrist
import frc.robot.subsystems.Elevator

class MoveL2 : Command() {
    val L2CommandSequence = SequentialCommandGroup(
        MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
        MoveElevator(Constants.ElevatorConstants.ElevatorState.L2.position)

    )
    override fun initialize() {
        ScheduleCommand(L2CommandSequence)
    }

    override fun isFinished(): Boolean {
        return L2CommandSequence.isFinished
        //fixme return Elevator.elevEncoder.equals(Constants.ElevatorConstants.ElevatorState.L2.position)
    }
}