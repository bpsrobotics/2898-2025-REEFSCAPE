package frc.robot.commands.Sequences
 // a simple startup protocol, or if u wanna move it back down to L1
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ScheduleCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.commands.wrist.MoveWrist
import frc.robot.commands.elevator.MoveElevator
import frc.robot.subsystems.Elevator

class StartupL1 : Command() {
    val commandGroup = SequentialCommandGroup(
        MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
        MoveElevator(Constants.ElevatorConstants.ElevatorState.Stow.position)
    )

    override fun initialize() {
        ScheduleCommand(commandGroup)
    }

    override fun isFinished(): Boolean {
        return commandGroup.isFinished
        //fixme return Elevator.elevEncoder.equals(Constants.ElevatorConstants.ElevatorState.Stow.position)
    }
}