package frc.robot.commands.elevator

import edu.wpi.first.units.Units.*

import edu.wpi.first.units.measure.MutDistance
import edu.wpi.first.units.measure.MutLinearVelocity
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.ScheduleCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Elevator.botLimit
import frc.robot.subsystems.Elevator.elevEncoder
import frc.robot.subsystems.Elevator.leftMaster
import frc.robot.subsystems.Elevator.topLimit

class ElevSysIDRoutine(val direction: Direction, val quasistaic: Boolean) : Command() {
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private val d_appliedVoltage: MutVoltage = MutVoltage(0.0,0.0, Volts)

    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private val d_distance: MutDistance = MutDistance(0.0,0.0, Meters)

    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private val d_velocity: MutLinearVelocity = MutLinearVelocity(0.0,0.0, MetersPerSecond)
    val routine = SysIdRoutine(
        SysIdRoutine.Config(),
        SysIdRoutine.Mechanism(
            { volts: Voltage -> Elevator.setVoltage(volts.`in`(Volts))
            },
            { log: SysIdRoutineLog -> log.motor(leftMaster.deviceId.toString())
                .voltage(d_appliedVoltage.mut_replace(Volts.of(leftMaster.busVoltage)))
                .linearPosition(d_distance.mut_replace(Meters.of(Elevator.getPos())))
                .linearVelocity(d_velocity.mut_replace(MetersPerSecond.of(elevEncoder.rate)))
            },
            Elevator
        )
    )
    lateinit var command: Command

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