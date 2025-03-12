package frc.robot.commands.wrist

import beaverlib.utils.Units.Angular.radiansPerSecond
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.units.measure.MutAngularVelocity

import edu.wpi.first.units.measure.MutDistance
import edu.wpi.first.units.measure.MutLinearVelocity
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.ScheduleCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import frc.robot.Constants
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Elevator.botLimit
import frc.robot.subsystems.Elevator.elevEncoder
import frc.robot.subsystems.Elevator.leftMaster
import frc.robot.subsystems.Elevator.topLimit
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.Wrist.armMotor
import frc.robot.subsystems.Wrist.deltaAngle
import frc.robot.subsystems.Wrist.encoder
import kotlin.math.abs

class WristSysID(val direction: Direction, val quasistaic: Boolean) : Command() {
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private val d_appliedVoltage: MutVoltage = MutVoltage(0.0,0.0, Volts)

    var timer = 0.0
    val sigmaTimer = Timer.getFPGATimestamp() //todo, rename what ever this is.

    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private val d_angle: MutAngle = MutAngle(0.0,0.0, Radians)

    fun topLimitWrist() :Boolean {
        return encoder.get() >= Constants.PivotConstants.UPPER_LIMIT
    }
    fun bottomLimitWrist() :Boolean{
        return encoder.get() <= Constants.PivotConstants.LOWER_LIMIT
    }
    fun calculateTimeDifferential():Double {
        val differenceTime = timer - sigmaTimer
        timer = sigmaTimer
        return abs(differenceTime)
    }
    fun tooMuchVoltage():Boolean {
        return armMotor.busVoltage >= 35.0
    }
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private val d_velocity: MutAngularVelocity = MutAngularVelocity(0.0,0.0, RadiansPerSecond)
    val routine = SysIdRoutine(
        SysIdRoutine.Config(),
        SysIdRoutine.Mechanism(
            { volts: Voltage -> Wrist.setVoltage(volts.`in`(Volts))
            },
            { log: SysIdRoutineLog -> log.motor(armMotor.deviceId.toString())
                .voltage(d_appliedVoltage.mut_replace(Volts.of(armMotor.busVoltage)))
                .angularPosition(d_angle.mut_replace(Rotations.of(encoder.get())))
                .angularVelocity(d_velocity.mut_replace(RadiansPerSecond.of(deltaAngle/calculateTimeDifferential()))) //fixme set a proper time instead of 0.1
            },
            Wrist
        )
    )
    lateinit var command: Command

    init {
        addRequirements(Wrist)
    }

    override fun initialize() {
        if (quasistaic) {
            command = routine.quasistatic(direction)
            ScheduleCommand(command)
        } else {
            command = routine.dynamic(direction)
            ScheduleCommand(command)
        }
    }

    override fun isFinished(): Boolean {
        if (direction == Direction.kForward) {
            return topLimitWrist() //todo, configure for when voltage goes above 35 volts.
        } else {
            return bottomLimitWrist()
        }
    }

    override fun end(interrupted: Boolean) {
        command.cancel()
    }

} // fixme(if this code doesn't work, return back to the original SYSID model, should be in a previous git commit u prolly gotta scroll)