package frc.robot.commands.wristcommands
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.Timer
import frc.robot.subsystems.Wrist
import frc.robot.Constants
import edu.wpi.first.math.trajectory.TrapezoidProfile


class WristCommands : Command() {
    val timer = Timer()
    var targetSpeed: Double = 0.0001

    /*init {
        this.wristcommands = Wrist
    }*/

    override fun initialize(){
        if (Wrist.setpoint !in Constants.PivotConstants.LOWER_LIMIT..Constants.PivotConstants.UPPER_LIMIT) return
        timer.restart()
        Wrist.curState = TrapezoidProfile.State(Wrist.getPos(), Wrist.encoder.get()) //todo configure encoder rate
        Wrist.goalState = TrapezoidProfile.State(Wrist.goalState.position, 0.0)
    } // todo rework this to fit the movements of the wrist

    override fun execute() {
        targetSpeed = Wrist.profile.calculate(timer.get(), Wrist.curState, Wrist.goalState).velocity
        Wrist.feedbackController(targetSpeed)
    }

    override fun isFinished(): Boolean {
        return targetSpeed == 0.0
    }


}