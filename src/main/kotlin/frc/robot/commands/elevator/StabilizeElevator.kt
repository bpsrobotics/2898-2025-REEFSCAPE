package frc.robot.commands.elevator

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Elevator

/** Command to keep the elevator at the current [Elevator.goalState] */
class StabilizeElevator() : Command() {
    val timer = Timer()
    init {addRequirements(Elevator)}
    override fun initialize() {}

    override fun execute() {
        Elevator.closedLoopPositionControl()
    }

    override fun isFinished(): Boolean { return false }
}