package frc.robot.commands.intake

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Intake.runMotor

class StopIntake() : Command() {
    init {
        addRequirements(Intake)
}
    override fun execute() {
        runMotor(0.0)
    }

    override fun isFinished(): Boolean {
        return false
    }
}