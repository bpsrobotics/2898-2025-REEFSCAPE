package frc.robot.commands.intake

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Intake

/**
 * Runs the intake Raw (Make sure
 */
class RunOuttake(val speed: Double) : Command() {
    override fun initialize() {  }
    override fun execute() {
        Intake.runMotor(speed)
    }

    override fun isFinished(): Boolean {
        return false
    }
}