package frc.robot.commands.wrist

import beaverlib.utils.Sugar.clamp
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Wrist.profiledPID
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.Wrist.pos
import frc.robot.subsystems.Wrist.rate
import frc.robot.subsystems.Wrist.velocity
import kotlin.math.PI

class MoveWrist(var goalPosition: Double) : Command() {
    val timer = Timer()
    init {addRequirements(Wrist)}
    override fun initialize() {
        profiledPID.reset(pos, rate)
        profiledPID.enableContinuousInput(-PI, PI)
        profiledPID.setTolerance(0.07)

    }

    override fun execute() {
        SmartDashboard.putNumber("/wrist/targ_pos", goalPosition)
        Wrist.profiledPIDControl(goalPosition)

    }

    override fun isFinished(): Boolean {
        return profiledPID.atGoal()
    }
}