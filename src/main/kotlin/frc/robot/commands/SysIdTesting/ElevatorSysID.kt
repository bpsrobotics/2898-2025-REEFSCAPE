package frc.robot.commands.SysIdTesting

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Volt
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.commands.elevator.VoltageElevator
import frc.robot.subsystems.Elevator


class ElevatorSysID : SubsystemBase() {
    val routine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(),
        SysIdRoutine.Mechanism(
            { volts: Voltage -> Elevator.leftMaster.setVoltage(volts.`in`(Volt))
            },
            { log: SysIdRoutineLog ->
                log.motor("Elevator")
                    .voltage(Units.Volts.of(Elevator.leftMaster.appliedOutput * Elevator.leftMaster.busVoltage))
                    .angularPosition(Units.Rotations.of(Elevator.leftMaster.encoder.position))
                    .current(Units.Amps.of(Elevator.leftMaster.outputCurrent))
                    .linearPosition(Units.Centimeters.of(Elevator.leftMaster.encoder.position))
            },
            this
        )
    )
    fun QuasistaticSysIDroutine(direction: SysIdRoutine.Direction): Command? {
        return routine.quasistatic(direction)
    }

    fun DynamicSysIDroutine(direction: SysIdRoutine.Direction): Command? {
        return routine.dynamic(direction)
    }
    fun GetSysIDVals(): Command{
        return SequentialCommandGroup(
            QuasistaticSysIDroutine(SysIdRoutine.Direction.kForward),
            WaitCommand(5.0),
            QuasistaticSysIDroutine(SysIdRoutine.Direction.kReverse),
            WaitCommand(5.0),
            DynamicSysIDroutine(SysIdRoutine.Direction.kForward),
            WaitCommand(5.0),
            DynamicSysIDroutine(SysIdRoutine.Direction.kReverse)
        )
    }
}