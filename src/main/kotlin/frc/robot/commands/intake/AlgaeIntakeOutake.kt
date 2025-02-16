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
        Intake.algaeIntake(OI.intakeSpeed * 3)
    }

    override fun isFinished(): Boolean {
        return Intake.hasCoral
    }
    fun inputStop(interrupted: Boolean) {
        Intake.intake(0.0)
    }
    fun removeCoral() {
        Intake.intake(OI.intakeSpeed * OI.intakeSpeed.sign)
    }
} //todo: check if this is redundant or not