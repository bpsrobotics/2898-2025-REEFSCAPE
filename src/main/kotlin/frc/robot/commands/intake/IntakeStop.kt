package frc.robot.commands.intake

import edu.wpi.first.wpilibj.Timer
import frc.robot.subsystems.Intake
import edu.wpi.first.wpilibj2.command.Command

class IntakeStop : Command() {
    val timer = Timer()
    init {addRequirements(Intake)}

        override fun initialize() {

        }

        override fun execute() {
            Intake.intake(0.0)
        }

        override fun isFinished(): Boolean {
            return false
        }
}