package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Intake

class RunIntake(val speed: Double = 0.35) : Command() {
    init {
        addRequirements(Intake)
    }
    override fun initialize() {}
    override fun execute() {
        Intake.runMotor(speed)
    }

    override fun isFinished(): Boolean {
//        return false
        return Intake.hasCoral.asBoolean
    }
}