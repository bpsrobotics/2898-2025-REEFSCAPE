package frc.robot.commands.wristcommands



import frc.robot.subsystems.Wrist
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.Timer

class ConfigureWrist: Command() {
    val timer = Timer()
    init { addRequirements(Wrist)}

    override fun initialize() {

    }

    override fun execute() {
        Wrist.feedbackController(0.0)
    }

    override fun isFinished(): Boolean {
        return false
    }

}