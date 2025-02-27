package frc.robot.commands.elevator

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Elevator
import java.util.function.DoubleSupplier

class VoltageElevator(val volts : DoubleSupplier, val time : Double) : Command() {
    val timer = Timer()
    var applied_volts = 0.0
    init {addRequirements(Elevator)}

    override fun initialize() { timer.restart() }
    override fun execute() {
        applied_volts = SmartDashboard.getNumber("/Elevator/Voltage", volts.asDouble)
        Elevator.leftMaster.set(applied_volts) }
    override fun isFinished(): Boolean { return timer.hasElapsed(time) }
}