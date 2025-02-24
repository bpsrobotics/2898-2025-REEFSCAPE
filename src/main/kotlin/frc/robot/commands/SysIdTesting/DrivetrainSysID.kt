package frc.robot.commands.SysIdTesting

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.units.measure.MutAngularVelocity
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ScheduleCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Drivetrain.swerveDrive
import frc.robot.subsystems.Elevator.botLimit
import frc.robot.subsystems.Elevator.topLimit
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.Wrist.armMotor
import frc.robot.subsystems.Wrist.deltaAngle
import frc.robot.subsystems.Wrist.encoder
import swervelib.SwerveModule


class DrivetrainSysID(val direction: Direction, val quasistaic: Boolean) : Command() {
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private val d_appliedVoltage: MutVoltage = MutVoltage(0.0,0.0, Volts)

    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private val d_angle: MutAngle = MutAngle(0.0,0.0, Radians)

    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private val d_velocity: MutAngularVelocity = MutAngularVelocity(0.0,0.0, RadiansPerSecond)


    val routine = SysIdRoutine(
        SysIdRoutine.Config(),
        SysIdRoutine.Mechanism(
            { volts: Voltage -> swerveDrive.modules.forEach {
                it.driveMotor.voltage = volts.`in`(Volt)
                }
            },
            { log: SysIdRoutineLog -> log.motor(armMotor.deviceId.toString())
                swerveDrive.modules.forEach {
                    Drivetrain.logDriveMotor(it, log)
                }
            },
            Drivetrain
        )
    )
    lateinit var command: Command

    init {
        addRequirements(Drivetrain)
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
            return topLimit.get()
        } else {
            return botLimit.get()
        }
    }

    override fun end(interrupted: Boolean) {
        command.cancel()
    }

}