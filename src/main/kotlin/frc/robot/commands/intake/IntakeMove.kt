package frc.robot.commands.intake

import edu.wpi.first.wpilibj.Timer
import frc.robot.subsystems.Intake
import edu.wpi.first.wpilibj2.command.Command

class IntakeMove: Command() {
    val timer = Timer()
    init {addRequirements(Intake)}

    override fun initialize() {
        timer.reset()
        timer.start()
    }

    override fun execute() { //todo these values need to be changed depending on how much voltage is needed
        if (!timer.hasElapsed(0.1)) {
            Intake.output = -0.4
        } else {
            if (!timer.hasElapsed(0.5)) {
                Intake.output = 0.7
            }
        }
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(1.0)
    }
     override fun end(interrupted: Boolean){
        Intake.intake(0.0)
     }
}