package frc.robot.commands.wrist

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Wrist

class StabilizeWrist : Command() {
    val timer = Timer()
    init {addRequirements(Wrist)}
    override fun initialize() {}

    override fun execute() {
        Wrist.closedLoopControl(0.0)
    }

    override fun isFinished(): Boolean { return false }
}