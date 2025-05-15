package frc.robot.commands.wrist

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.Wrist.pid

class StabilizeWrist : Command() {
    val timer = Timer()
    init {addRequirements(Wrist)}
    override fun initialize() {
        pid.reset()
    }

    override fun execute() {
        Wrist.closedLoopPositionControl()
    }

    override fun isFinished(): Boolean { return false }
}