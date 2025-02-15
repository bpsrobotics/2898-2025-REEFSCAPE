package frc.robot.commands.intake

import edu.wpi.first.wpilibj.Timer
import frc.robot.subsystems.Intake
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants.IntakeConstants.INTAKE
import frc.robot.Constants.IntakeConstants.OUTTAKE

class CoralIntake (): Command() {
    val timer = Timer()
    init { //addRequirements isn't working after import for some reason
        addRequirements(Intake)
    }

    override fun initialize() {
        timer.reset()
        timer.start()
    }

    override fun execute() { //todo these values need to be changed for coral
        if (!timer.hasElapsed(0.1)) {
            INTAKE // postive and has more voltage to take in algae easier
        } else {
            if (!timer.hasElapsed(0.5)) {
                OUTTAKE // negative need less voltage to spit out algae
            }
        }
    }

    override fun isFinished(): Boolean {
        return Intake.hasCoral
    }
    override fun end(interrupted: Boolean){
        Intake.intake(0.0)
    }
}