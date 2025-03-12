package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.OI
import frc.robot.subsystems.TheFabledStateSpace

class StateSpaceActivate: Command() {
    override fun initialize() {
        addRequirements(TheFabledStateSpace)
        TheFabledStateSpace.setNewVoltage(0.0)
    }
    override fun execute() {
        TheFabledStateSpace.setNewVoltage(OI.intakeSpeed)
    }

    override fun isFinished(): Boolean {
        return isFinished
    }
}