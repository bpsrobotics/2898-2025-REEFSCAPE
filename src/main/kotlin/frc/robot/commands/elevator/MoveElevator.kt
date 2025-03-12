package frc.robot.commands.elevator

import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Elevator.getPos
import frc.robot.subsystems.Elevator.profiledPID
import frc.robot.subsystems.Wrist

/** Command to move the elevator to a [goalPosition], finishes once the trapezoid profile has finished
 * @param goalPosition Position to move to.
 * */
class MoveElevator(val goalPosition : Double) : Command() {
    init {addRequirements(Elevator)}
    override fun initialize() {
//        if (goalPosition !in Constants.ElevatorConstants.LOWER_LIMIT..Constants.ElevatorConstants.UPPER_LIMIT ) return
        profiledPID.reset(getPos())
        profiledPID.setTolerance(0.01)
    }

    override fun execute() {
        println("moving elev to  " + goalPosition)
        Elevator.profiledPIDControl(goalPosition)
    }

    override fun isFinished(): Boolean {
        return profiledPID.atGoal()
    }


}

