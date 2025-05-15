package frc.robot.commands.intake

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Intake
import kotlin.concurrent.timer

/**
 * Runs the intake Raw (Make sure
 */
class RunOuttake(val speed: Double, val time: Double = 20.0) : Command() {
    val timer = Timer()
    init {
        addRequirements(Intake)
    }
    override fun initialize() {
        timer.restart()
    }
    override fun execute() {
        println("run outtake $speed")
        Intake.runMotor(speed)
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(time)
    }
}