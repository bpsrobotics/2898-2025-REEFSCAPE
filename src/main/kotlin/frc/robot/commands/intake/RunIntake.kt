package frc.robot.commands.intake

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Intake

class RunIntake(val speed: Double = 0.2, val currentAverageThreshold : Double = 0.5) : Command() {//todo The current the motor draws when coral is intaked
    val gracePeriod = Timer()
    override fun initialize() { gracePeriod.restart() }
    override fun execute() {
        Intake.runMotor(speed)
    }

    override fun isFinished(): Boolean {
        return Intake.buffer.calculate(Intake.currentAverage >= currentAverageThreshold) && gracePeriod.hasElapsed(0.5)
    }
}