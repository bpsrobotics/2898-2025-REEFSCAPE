package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Intake

class StopIntake() : Command() {
    init {
        addRequirements(Intake)
}
    override fun execute() {
        Intake.stop()
    }

    override fun isFinished(): Boolean {
        return false
    }
}