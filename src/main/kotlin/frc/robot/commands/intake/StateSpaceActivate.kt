package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.OI
import frc.robot.subsystems.TheFabledStateSpace

class StateSpaceActivate: Command() {
    override fun initialize() {
        addRequirements(TheFabledStateSpace)
        TheFabledStateSpace.input = 0.0
        TheFabledStateSpace.setNewVoltage()
    }
    override fun execute() {
        TheFabledStateSpace.input = OI.intakeSpeed
        TheFabledStateSpace.setNewVoltage()
    }

    override fun isFinished(): Boolean {
        return isFinished
    }
}