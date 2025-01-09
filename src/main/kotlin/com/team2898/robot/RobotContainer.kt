// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team2898.robot

//import com.team2898.robot.Constants.OperatorConstants

import com.pathplanner.lib.auto.AutoBuilder

import com.team2898.engine.utils.Vector
import com.team2898.robot.OI.driverX
import com.team2898.robot.OI.intakeSpeed
import com.team2898.robot.OI.operatorTrigger
import com.team2898.robot.OI.resetGyro
import com.team2898.robot.OI.rightTrigger
import com.team2898.robot.OI.translationX
import com.team2898.robot.OI.translationY
import com.team2898.robot.OI.turnX
import com.team2898.robot.commands.bintake.Grasp
import com.team2898.robot.commands.bintake.RunBintake
import com.team2898.robot.commands.intake.RunIntake
import com.team2898.robot.commands.swerve.NavXReset
import com.team2898.robot.subsystems.*
import com.team2898.robot.subsystems.Drivetrain.getDriveSysIDCommand
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.swerve.TeleopDriveCommand


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems and commands are defined here...
    //private val m_exampleSubsystem = ExampleSubsystem()
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private val driverController = XboxController(0)

    private var autoCommandChooser: SendableChooser<Command> = SendableChooser()

    val teleopDrive: TeleopDriveCommand =
        TeleopDriveCommand(
            { MathUtil.applyDeadband(-translationY, 0.1) },
            { MathUtil.applyDeadband(-translationX, 0.1) },
            { MathUtil.applyDeadband(turnX, 0.1)},
            { true },
            { false },
        )


    val intake: RunIntake = RunIntake({MathUtil.applyDeadband(-intakeSpeed, 0.5)})


    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        initializeObjects()

        // Configure the trigger bindings

        autoCommandChooser = AutoBuilder.buildAutoChooser("6piece")

        Drivetrain.defaultCommand = teleopDrive
        Intake.defaultCommand = intake

        configureBindings()

        SmartDashboard.putData("Auto mode", autoCommandChooser)



    }
    fun getAutonomousCommand(): Command{
        val path = autoCommandChooser.selected
        return path
    }

    private fun initializeObjects() {
        Drivetrain
        Intake
        ToteManipulator
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [Trigger.Trigger] constructor with an arbitrary
     * predicate, or via the named factories in [ ]'s subclasses for [ ]/[ PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller] controllers or [Flight][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        //Trigger { m_exampleSubsystem.exampleCondition() }
        //        .onTrue(ExampleCommand(m_exampleSubsystem))

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
        //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand())

        when {
            driverX -> {
                getDriveSysIDCommand()
            }

            operatorTrigger -> {
                Grasp()
            }

            resetGyro -> {
                NavXReset()
            }
        }
        when (OI.hatVector) {
            Vector(0, -1) -> Intake.getIntakeSysIDCommand()
            Vector(0,1) -> RunBintake({5.0})
        }



    }

}