package frc.robot.commands.wrist

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Wrist.coralState

class ToggleState : Command() {
    val timer = Timer()
    override fun initialize() {
        timer.restart()
        coralState = !coralState
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(0.01)
    }
}