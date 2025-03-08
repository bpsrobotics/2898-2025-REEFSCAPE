package frc.robot.commands.wrist

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.Wrist.pos
import frc.robot.subsystems.Wrist.profiledPID
import frc.robot.subsystems.Wrist.rate

/**
 * @param speed IS A PERCENTAGE, NOT VOLTAGE
 */
class VoltageWrist(val speed : Double = 0.01) : Command(){
    val timer = Timer()
    init {addRequirements(Wrist)}
    override fun initialize() {
        profiledPID.reset(pos)

    }

    override fun execute() {
        Wrist.setVoltage(Wrist.feedForward.calculate(pos, rate) + speed)
    }

    override fun isFinished(): Boolean { return false }

//    override fun cancel() {
//        Wrist.setVoltage(0.0)
//        Wrist.armMotor.stopMotor()
//    }
}