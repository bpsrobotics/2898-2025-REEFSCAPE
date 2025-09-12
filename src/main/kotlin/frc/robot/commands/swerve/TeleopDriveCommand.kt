// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.swerve

import edu.wpi.first.math.geometry.Transform2d
import beaverlib.utils.Sugar.clamp
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Elevator
import swervelib.SwerveController
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import kotlin.math.pow

/**
 * A command that controls the swerve drive using joystick inputs.
 * @param vForward The x velocity of the robot.
 * @param vStrafe The y velocity of the robot.
 * @param omega The angular velocity of the robot.
 * @param driveMode Boolean supplier that returns true if the robot should drive in field-oriented mode.
 * @param slowMode Boolean supplier that returns true if the robot should drive in slow mode.
 * @see Drivetrain
 */
class TeleopDriveCommand(
    vForward: () -> Double,
    vStrafe: () -> Double,
    omega: () -> Double,
    driveMode: () -> Boolean,
    slowMode: () -> Double
) : Command() {
    private val vForward: () -> Double
    private val vStrafe: () -> Double
    private val omega: () -> Double
    private val driveMode: () -> Boolean
    private val slowMode: () -> Double
    private val controller: SwerveController
    private val swerve: Drivetrain = Drivetrain
    var addSpeed = Transform2d()
    /** adds to the speed of the robot */
    val speedConsumer: (Transform2d) -> Unit = { addSpeed += it}

    init {
        this.vForward = vForward
        this.vStrafe = vStrafe
        this.omega = omega
        this.driveMode = driveMode
        this.slowMode = slowMode
        controller = swerve.getSwerveController()
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve)
    }

    /** @suppress */
    override fun initialize() {}

    /** @suppress */
    override fun execute() {
        var forwardVelocity = vForward()
        var strafeVelocity = vStrafe()
        var angVelocity = omega()
        var slowMode = slowMode()


        forwardVelocity *= (1.0 - (slowMode.pow(3) * 0.8))

        strafeVelocity *= (1.0 - (slowMode.pow(3) * 0.8))
        angVelocity *= (1.0 - (slowMode.pow(3) * 0.85))
        SmartDashboard.putNumber("vX", forwardVelocity)
        SmartDashboard.putNumber("vY", strafeVelocity)
        SmartDashboard.putNumber("omega", angVelocity)
        SmartDashboard.putNumber("slowmode", slowMode)


        // Drive using raw values.
//        swerve.drive(
//            Translation2d(forwardVelocity * swerve.maximumSpeed+addSpeed.translation.x, strafeVelocity * swerve.maximumSpeed+addSpeed.translation.y),
//            -angVelocity * controller.config.maxAngularVelocity+addSpeed.rotation.degrees,
//            driveMode()
//        )
        addSpeed = Transform2d()
//        swerve.drive(
//            Translation2d(forwardVelocity * swerve.maximumSpeed, strafeVelocity * swerve.maximumSpeed),
//            angVelocity * controller.config.maxAngularVelocity,
//            true
//        )
        swerve.driveFieldOriented(ChassisSpeeds(forwardVelocity * swerve.maximumSpeed + addSpeed.translation.x, strafeVelocity * swerve.maximumSpeed + addSpeed.translation.y, angVelocity * swerve.maxAngularSpeed))
    }

    /** @suppress */
    override fun end(interrupted: Boolean) {}

    /** @suppress */
    override fun isFinished(): Boolean {
        return false
    }
}