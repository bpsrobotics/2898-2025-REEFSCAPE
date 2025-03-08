package frc.robot.commands.sequence

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.commands.elevator.MoveElevator
import frc.robot.commands.wrist.MoveWrist
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.Wrist.coralState
import frc.robot.subsystems.Wrist.presetGoal

class Stow : Command() {
    val commandGroup = if (Wrist.isObstructing) {
        SequentialCommandGroup(MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
            MoveElevator(Constants.ElevatorConstants.ElevatorState.Stow.position),
            MoveWrist(Constants.PivotConstants.PivotState.Stow.position)
        )
    } else {
        SequentialCommandGroup(

            MoveElevator(Constants.ElevatorConstants.ElevatorState.Stow.position),
            MoveWrist(Constants.PivotConstants.PivotState.Stow.position)
        )
    }
    override fun initialize() {
        presetGoal = Constants.PivotConstants.PivotState.Stow
        coralState = true
        commandGroup.schedule()
    }

    override fun isFinished(): Boolean {
        return commandGroup.isFinished
    }
}