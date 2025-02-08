package frc.robot.commands.wrist

import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Wrist

class MoveWrist(val goalPosition: Double) : Command() {
    val timer = Timer()
    var targetSpeed : Double = 0.0001
    init {addRequirements(Wrist)}
    override fun initialize() {
        if (goalPosition !in Constants.PivotConstants.LOWER_LIMIT..Constants.PivotConstants.UPPER_LIMIT) return
        timer.restart()
        Wrist.currentState = TrapezoidProfile.State(Wrist.getPos(), Wrist.deltaAngle / 0.02)
        Wrist.goalState = TrapezoidProfile.State(goalPosition, 0.0)
    }

    override fun execute() {
        targetSpeed = Wrist.profile.calculate(timer.get(), Wrist.currentState, Wrist.goalState).velocity
        Wrist.closedLoopControl(targetSpeed)
    }

    override fun isFinished(): Boolean {
        return targetSpeed == 0.0
    }
}