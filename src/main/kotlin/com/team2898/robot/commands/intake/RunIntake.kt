package com.team2898.robot.commands.intake

import com.team2898.robot.subsystems.Intake
import edu.wpi.first.wpilibj2.command.Command
import java.util.function.DoubleSupplier

class RunIntake(val speed: DoubleSupplier) : Command() {

    private val intakeSubsystem = Intake

    init {
        addRequirements(Intake)
    }

    override fun execute() {
        intakeSubsystem.intakeMotor.set(speed.asDouble)
    }

    override fun isFinished(): Boolean {
        return false
    }
}