package frc.robot.commands.elevator

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Elevator
import java.util.function.DoubleSupplier

class VoltageElevator(val volts : DoubleSupplier, val time : Double) : Command() {
    val timer = Timer()
    init {addRequirements(Elevator)}

    override fun initialize() { timer.restart() }
    override fun execute() {
        println(volts.asDouble)
        Elevator.leftMaster.set(volts.asDouble) }
    override fun isFinished(): Boolean { return timer.hasElapsed(time) }
}