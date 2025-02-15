package frc.robot.commands.intake

import edu.wpi.first.wpilibj.Timer
import frc.robot.subsystems.Intake
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.OI

class RunIntake : Command() {
    val timer = Timer()
    init {addRequirements(Intake)}

    override fun initialize() {
        Intake.intake(0.5)
    }

    override fun execute() {
        Intake.intake(OI.intakeSpeed)
    }
    override fun isFinished(): Boolean {
        return Intake.hasCoral
    }
    fun inputStop(interrupted: Boolean) {
        Intake.intake(0.0)
    }
    fun removeCoral() {
        Intake.intake(OI.intakeSpeed)
    }
}