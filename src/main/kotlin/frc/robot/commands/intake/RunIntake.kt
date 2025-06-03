package frc.robot.commands.intake

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Intake

class RunIntake(val speed: Double = 0.35) : Command() {
    private val timer = Timer()
    private var done = false
    init {
        addRequirements(Intake)
    }
    override fun initialize() {}
    override fun execute() {
        if (Intake.hasCoral.asBoolean) {
            if (timer.isRunning && timer.hasElapsed(Constants.IntakeConstants.STOP_TIME)) {
                Intake.stop()
                done = true
                return
            }
            else timer.start()
        }
        Intake.runMotor(speed)
    }

    override fun isFinished(): Boolean {
//        return false
        return done
    }
}