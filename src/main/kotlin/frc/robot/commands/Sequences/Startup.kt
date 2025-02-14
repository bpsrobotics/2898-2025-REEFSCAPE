package frc.robot.commands.Sequences
 // a simple startup protocol, or if u wanna move it back down to L1
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.ScheduleCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.commands.wrist.MoveWrist
import frc.robot.commands.elevator.MoveElevator

class Startup : Command() {
    val commandGroup = SequentialCommandGroup(
        MoveWrist(Constants.PivotConstants.Traverse.position),
        MoveElevator(Constants.ElevatorConstants.LOWER_LIMIT)
    )

    override fun initialize() {
        ScheduleCommand(commandGroup)
    }

    override fun isFinished(): Boolean {
        return false //fixme finish this, there is no real error.
    }
}