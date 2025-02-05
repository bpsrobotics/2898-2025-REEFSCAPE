package frc.robot.commands.elevator

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Elevator

class VoltageElevator(val volts : Double, val time : Double) : Command() {
    val timer = Timer()
    init {addRequirements(Elevator)}

    override fun initialize() { timer.restart() }
    override fun execute() { Elevator.leftMaster.set(volts) }
    override fun isFinished(): Boolean { return timer.hasElapsed(time) }
}