package frc.robot.commands.wrist

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.Wrist.coralState
import frc.robot.subsystems.Wrist.presetGoal

class RetractWrist : Command() {
    lateinit var command: Command

    override fun initialize() {
        if (coralState) {
            command = MoveWrist(presetGoal.coralRetract().position)
            presetGoal = presetGoal.coralExtend()
        } else {
            command = MoveWrist(presetGoal.algaeRetract().position)
            presetGoal = presetGoal.algaeExtend()
        }
        command.schedule()
    }

    override fun isFinished(): Boolean {
        return command.isFinished
    }
}