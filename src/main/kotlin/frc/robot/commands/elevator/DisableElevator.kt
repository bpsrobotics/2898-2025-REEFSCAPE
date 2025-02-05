package frc.robot.commands.elevator

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Elevator

/** Command that sets the elevator motors to not move
 * NOTE: Command does not end naturally, it must be interrupted  */
class DisableElevator() : Command() {
    init {addRequirements(Elevator)}
    override fun execute() { Elevator.leftMaster.set(0.0) }
    override fun isFinished(): Boolean { return false }
}