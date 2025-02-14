package frc.robot.commands.elevator

import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Wrist

/** Command to move the elevator to a [goalPosition], finishes once the trapezoid profile has finished
 * @param goalPosition Position to move to.
 * */
class MoveElevator(val goalPosition : Double) : Command() {
    val timer = Timer()
    var targetSpeed : Double = 0.0001
    init {addRequirements(Elevator)}
    override fun initialize() {
        if (goalPosition !in Constants.ElevatorConstants.LOWER_LIMIT..Constants.ElevatorConstants.UPPER_LIMIT || Wrist.isObstructing()) return
        timer.restart()
        Elevator.currentState = TrapezoidProfile.State(Elevator.getPos(), Elevator.elevEncoder.rate)
        Elevator.goalState = TrapezoidProfile.State(goalPosition, 0.0)
    }

    override fun execute() {
        targetSpeed = Elevator.profile.calculate(timer.get(), Elevator.currentState, Elevator.goalState).velocity
        Elevator.closedLoopMotorControl(goalPosition)
    }

    override fun isFinished(): Boolean {
        return targetSpeed == 0.0
    }
}