package frc.robot.commands.wrist

import beaverlib.utils.Sugar.clamp
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Wrist.profiledPID
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.Wrist.pid
import frc.robot.subsystems.Wrist.pos
import frc.robot.subsystems.Wrist.rate
import frc.robot.subsystems.Wrist.velocity
import kotlin.math.PI

class MoveWristBy(var goalPosition: Double) : Command() {
    val timer = Timer()
    init {addRequirements(Wrist)}
    override fun initialize() {
        profiledPID.reset(pos, rate)
        profiledPID.enableContinuousInput(-PI, PI)
        profiledPID.setTolerance(0.07)
        goalPosition += pos
    }

    override fun execute() {
        Wrist.profiledPIDControl(goalPosition)
        SmartDashboard.putNumber("targ_pos", profiledPID.setpoint.position)


    }

    override fun isFinished(): Boolean {
        return profiledPID.atGoal()
    }
}