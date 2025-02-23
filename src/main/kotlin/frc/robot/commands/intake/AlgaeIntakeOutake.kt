package frc.robot.commands.intake

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.OI
import frc.robot.subsystems.Intake
import kotlin.math.sign

class AlgaeIntakeOutake: Command() {
    val timer = Timer()
    init {addRequirements(Intake) }

    override fun initialize() {
        Intake.algaeIntake(0.0)
    }

    override fun execute() {
        Intake.algaeIntake(OI.useIntake * 3)
    }

    override fun isFinished(): Boolean {
        return Intake.hasCoral
    }
    fun inputStop(interrupted: Boolean) {
        Intake.intake(0.0)
    }
} //todo: check if this file is redundant or not