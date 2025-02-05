package frc.robot.commands.elevator

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Elevator

class ElevMove(private val goal: Double) : Command() {

    constructor(goal: Constants.ElevatorConstants.ElevatorState) : this(goal.position)

    override fun initialize() {
        Elevator.setGoal(goal)
    }

    override fun isFinished(): Boolean {
        return Elevator.isMoving()
    }


}