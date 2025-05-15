package frc.robot.commands.elevator

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Elevator
import java.util.function.DoubleSupplier

class VoltageElevator(val volts : () -> Double, val time : Double) : Command() {
    val timer = Timer()
    var applied_volts = 0.0
    init {addRequirements(Elevator)}

    override fun initialize() { timer.restart() }
    override fun execute() {
//        println(volts())
//        println(SmartDashboard.getNumber("/Elevator/Voltage", -1.0))
//        applied_volts = SmartDashboard.getNumber("/Elevator/Voltage", volts())
        Elevator.leftMaster.set(0.03) }
    override fun isFinished(): Boolean { return timer.hasElapsed(time) }
}