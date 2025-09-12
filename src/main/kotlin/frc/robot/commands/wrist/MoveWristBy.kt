package frc.robot.commands.wrist

import beaverlib.utils.Sugar.clamp
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.Constants.PivotConstants.LOWER_LIMIT
import frc.robot.Constants.PivotConstants.UPPER_LIMIT
import frc.robot.subsystems.Wrist.profiledPID
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.Wrist.pid
import frc.robot.subsystems.Wrist.pos
import frc.robot.subsystems.Wrist.rate
import frc.robot.subsystems.Wrist.velocity
import kotlin.math.PI

class MoveWristBy(var goalPosition: Double) : Command() {
    val timer = Timer()
    var newGoal = pos
    init {addRequirements(Wrist)}
    override fun initialize() {
        profiledPID.reset(pos, rate)
        profiledPID.enableContinuousInput(-PI, PI)
        profiledPID.setTolerance(0.01)
    }

    override fun execute() {
        if ((goalPosition + profiledPID.goal.position) in UPPER_LIMIT..LOWER_LIMIT) {
            newGoal = goalPosition + profiledPID.goal.position
        }
        Wrist.profiledPIDControl(newGoal)
        SmartDashboard.putNumber("/wrist/targ_pos", newGoal)


    }

    override fun isFinished(): Boolean {
        return profiledPID.atGoal()
    }
}