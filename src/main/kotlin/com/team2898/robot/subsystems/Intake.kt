package com.team2898.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax


import com.team2898.robot.RobotMap.IntakeId
import com.team2898.robot.subsystems.Drivetrain.getDriveSysIDRoutine
import com.team2898.robot.subsystems.ToteManipulator.motors
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Volt
import edu.wpi.first.units.Voltage

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.engine.utils.initMotorControllers
import kotlin.math.sign

object Intake : SubsystemBase() {
    val intakeMotor = CANSparkMax(IntakeId, CANSparkLowLevel.MotorType.kBrushless)
    val ks = 0.0
    val kv = 0.0
    val ff = SimpleMotorFeedforward(ks, kv)
    val pid = PIDController(0.0,0.0,0.0)
    var intakeSpeed = 0.0

    init {
        intakeMotor.restoreFactoryDefaults()
        intakeMotor.setSmartCurrentLimit(30)
        intakeMotor.idleMode = CANSparkBase.IdleMode.kBrake
        intakeMotor.inverted = true
        intakeMotor.burnFlash()



    }

    /**
     * The system identification routine for the launcher subsystem.
     */
    val routine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(),
        SysIdRoutine.Mechanism(
            { volts: Measure<Voltage> ->
                intakeMotor.setVoltage(volts.`in`(Volt))
            },
            { log: SysIdRoutineLog ->
                log.motor("intake")
                    .voltage(Units.Volts.of(intakeMotor.appliedOutput * intakeMotor.busVoltage))
                    .angularPosition(Units.Rotations.of(intakeMotor.encoder.position))
                    .angularVelocity(Units.RPM.of(intakeMotor.encoder.velocity))
                    .current(Units.Amps.of(intakeMotor.outputCurrent))
            },
            this
        )
    )

    /**
     * Creates a command to run a dynamic system identification routine
     * @param direction The direction to run the routine in
     * @return The command to run the routine
     */
    fun dynamicSysIDRoutine(direction: SysIdRoutine.Direction): Command? {
        return routine.dynamic(direction)
    }

    /**
     * Creates a command to run a quasi-static system identification routine
     * @param direction The direction to run the routine in
     * @return The command to run the routine
     */
    fun quasiStaticSysIDRoutine(direction: SysIdRoutine.Direction): Command? {
        return routine.quasistatic(direction)
    }

    /**
     * Generate a full command to SysID the intake
     * @return A command that SysIDs the intake
     */
    fun getIntakeSysIDCommand(): Command {
        return SequentialCommandGroup(
            dynamicSysIDRoutine(SysIdRoutine.Direction.kForward),
            WaitCommand(2.0),
            dynamicSysIDRoutine(SysIdRoutine.Direction.kReverse),
            WaitCommand(2.0),
            quasiStaticSysIDRoutine(SysIdRoutine.Direction.kForward),
            WaitCommand(2.0),
            quasiStaticSysIDRoutine(SysIdRoutine.Direction.kReverse)
        )
    }

    fun runIntake(speed: Double) {
        intakeSpeed = ff.calculate(speed) + pid.calculate(speed)
    }






}