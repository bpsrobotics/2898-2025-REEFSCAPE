package frc.robot.commands.wrist

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Wrist

class StopWrist : Command() {
    val timer = Timer()
    init {addRequirements(Wrist)}
    override fun initialize() {}

    override fun execute() {
        Wrist.armMotor.stopMotor()
    }

    override fun isFinished(): Boolean { return false }
}