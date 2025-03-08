package frc.robot.commands.intake

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Intake

class RunIntake(val speed: Double = 0.4, val currentAverageThreshold : Double = 10.0) : Command() {//todo The current the motor draws when coral is intaked
    val gracePeriod = Timer()
    init {
        addRequirements(Intake)
    }
    override fun initialize() {
        gracePeriod.restart()
    }
    override fun execute() {
        Intake.runMotor(speed)
    }

    override fun isFinished(): Boolean {
//        return false
        return Intake.buffer.calculate(Intake.hasCoral)
    }
}