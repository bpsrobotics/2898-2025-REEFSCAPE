// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

//import com.team2898.robot.Constants.OperatorConstants

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import frc.robot.OI.resetGyro
import frc.robot.OI.translationX
import frc.robot.OI.translationY
import frc.robot.OI.turnX
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.Sequences.StartupL1
import frc.robot.commands.Sequences.MoveL2
import frc.robot.commands.Sequences.MoveL3
import frc.robot.commands.Sequences.MoveL4
import frc.robot.commands.Sequences.MoveToIntake
import frc.robot.commands.elevator.DisableElevator
import frc.robot.commands.elevator.MoveElevator
import frc.robot.commands.elevator.StabilizeElevator
import frc.robot.commands.intake.AlgaeIntakeOutake
import frc.robot.commands.intake.CoralIntake
import frc.robot.commands.intake.RunIntake
import frc.robot.commands.swerve.NavXReset
import frc.robot.commands.swerve.TeleopDriveCommand
import frc.robot.subsystems.Drivetrain


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

    private var autoCommandChooser: SendableChooser<Command> = SendableChooser()



    val teleopDrive: TeleopDriveCommand =
        TeleopDriveCommand(
            { MathUtil.applyDeadband(-translationY, 0.1) },
            { MathUtil.applyDeadband(-translationX, 0.1) },
            { MathUtil.applyDeadband(-turnX, 0.1)},
            { true },
            { false }
        )

    val navXResetCommand: NavXReset = NavXReset()




    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {

        NamedCommands.registerCommand("coralintake", CoralIntake())
        NamedCommands.registerCommand("algaeintake", AlgaeIntakeOutake())
        NamedCommands.registerCommand("L1", MoveElevator(Constants.ElevatorConstants.ElevatorState.Stow.position))
        NamedCommands.registerCommand("L2", MoveElevator(Constants.ElevatorConstants.ElevatorState.L2.position))
        NamedCommands.registerCommand("L3", MoveElevator(Constants.ElevatorConstants.ElevatorState.L3.position))
        NamedCommands.registerCommand("L4", MoveElevator(Constants.ElevatorConstants.ElevatorState.L4.position))
        NamedCommands.registerCommand("disable", DisableElevator())
        NamedCommands.registerCommand("stabilize", StabilizeElevator())
        initializeObjects()

        // Configure the trigger bindings

        autoCommandChooser = AutoBuilder.buildAutoChooser("Left mid start")

        Drivetrain.defaultCommand = teleopDrive

        configureBindings()

        //SmartDashboard.putData("Auto mode", autoCommandChooser)



    }
    fun getAutonomousCommand(): Command{
        val path = autoCommandChooser.selected
        return path
    }

    private fun initializeObjects() {
        Drivetrain
//        Wrist
//        Elevator
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

        // MoveL# Sequences
        when {
            OI.moveL1 -> StartupL1().schedule()
            OI.moveL2 -> MoveL2().schedule()
            OI.moveL3 -> MoveL3().schedule()
            OI.moveL4 -> MoveL4().schedule()
            OI.moveToIntake -> MoveToIntake().schedule()
        }

        resetGyro.whileTrue(navXResetCommand)
    }
}