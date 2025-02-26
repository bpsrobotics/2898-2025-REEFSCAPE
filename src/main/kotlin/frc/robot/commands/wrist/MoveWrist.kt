package frc.robot.commands.wrist

import beaverlib.utils.Sugar.clamp
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Wrist

class MoveWrist(var goalPosition: Double) : Command() {
    val timer = Timer()
    var targetSpeed : Double = 0.0001
    init {addRequirements(Wrist)}
    override fun initialize() {
        goalPosition = goalPosition.clamp(Constants.PivotConstants.LOWER_LIMIT, Constants.PivotConstants.UPPER_LIMIT)
        timer.restart()
        Wrist.currentState = TrapezoidProfile.State(Wrist.getPos(), Wrist.deltaAngle / 0.02)
        Wrist.goalState = TrapezoidProfile.State(goalPosition, 0.0)
        Wrist.pid.setpoint = goalPosition
    }

    override fun execute() {
        targetSpeed = Wrist.profile.calculate(timer.get(), Wrist.currentState, Wrist.goalState).velocity
        Wrist.closedLoopControl(targetSpeed)
    }

    override fun isFinished(): Boolean {
        return targetSpeed == 0.0
    }
}