package frc.robot.commands.wrist

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Wrist

/**
 * @param speed IS A PERCENTAGE, NOT VOLTAGE
 */
class VoltageWrist(val speed : Double = 0.01) : Command(){
    val timer = Timer()
    init {addRequirements(Wrist)}
    override fun initialize() {}

    override fun execute() {
        println(speed)
        if(speed == 0.0) { }
        Wrist.setVoltage(speed * 12)
        println(speed)
    }

    override fun isFinished(): Boolean { return false }
    override fun end(interrupted: Boolean) {
        Wrist.setVoltage(0.0)
        Wrist.armMotor.stopMotor()
    }
//    override fun cancel() {
//        Wrist.setVoltage(0.0)
//        Wrist.armMotor.stopMotor()
//    }
}