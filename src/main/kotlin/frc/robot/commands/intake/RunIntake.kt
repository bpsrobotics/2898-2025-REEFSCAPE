package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Intake
import java.util.function.DoubleSupplier

class RunIntake(val speed: DoubleSupplier) : Command() {

    init {
        addRequirements(Intake)
    }

    override fun execute() {
        Intake.runMotor(speed.asDouble)
    }

    override fun isFinished(): Boolean {
        return false
    }

}