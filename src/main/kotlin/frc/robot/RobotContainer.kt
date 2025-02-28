// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

//import com.team2898.robot.Constants.OperatorConstants

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import frc.robot.OI.translationX
import frc.robot.OI.translationY
import frc.robot.OI.turnX
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.OI.highHatBack
import frc.robot.OI.highHatForward
import frc.robot.OI.resetGyro
import frc.robot.OI.sysidBD
import frc.robot.OI.sysidBQ
import frc.robot.OI.sysidFD
import frc.robot.OI.sysidFQ
import frc.robot.commands.elevator.*
import frc.robot.commands.intake.RunIntake
import frc.robot.commands.intake.RunOuttake
import frc.robot.commands.sequence.PositionL2
import frc.robot.commands.swerve.NavXReset
import frc.robot.commands.swerve.TeleopDriveCommand
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Wrist.SysIDWrist

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
        NamedCommands.registerCommand("coralintake", RunIntake(0.2, 0.5)) //Todo set this properly
        //todo NamedCommands.registerCommand("algaeintake", AlgaeIntakeOutake())
        NamedCommands.registerCommand("L1", MoveElevator(Constants.ElevatorConstants.ElevatorState.Stow.position))
        NamedCommands.registerCommand("L2", MoveElevator(Constants.ElevatorConstants.ElevatorState.L2.position))
        NamedCommands.registerCommand("L3", MoveElevator(Constants.ElevatorConstants.ElevatorState.L3.position))
        NamedCommands.registerCommand("L4", MoveElevator(Constants.ElevatorConstants.ElevatorState.L4.position))
        NamedCommands.registerCommand("disable", DisableElevator())
        NamedCommands.registerCommand("stabilize", StabilizeElevator())
        initializeObjects()

        // Configure the trigger bindings

//        autoCommandChooser = AutoBuilder.buildAutoChooser("Basic")

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
        Elevator
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

        resetGyro.whileTrue(navXResetCommand)

//        sysidFQ.whileTrue(SysIDWrist(SysIdRoutine.Direction.kForward, true))
//        sysidBQ.whileTrue(SysIDWrist(SysIdRoutine.Direction.kReverse, true))
//        sysidFD.whileTrue(SysIDWrist(SysIdRoutine.Direction.kForward, false))
//        sysidBD.whileTrue(SysIDWrist(SysIdRoutine.Direction.kReverse, false))

//        sysidFQ.whileTrue(SysIDElev(SysIdRoutine.Direction.kForward, true))
//        sysidBQ.whileTrue(SysIDElev(SysIdRoutine.Direction.kReverse, true))
//        sysidFD.whileTrue(SysIDElev(SysIdRoutine.Direction.kForward, false))
//        sysidBD.whileTrue(SysIDElev(SysIdRoutine.Direction.kReverse, false))

        highHatForward.whileTrue(RunIntake()) //TODO set values properly
        highHatBack.whileTrue(RunOuttake(-0.2))
//        OI.moveL1.onTrue(StartupL1())
        OI.moveL2.onTrue(MoveElevator(Constants.ElevatorConstants.ElevatorState.L2.position))
//        OI.moveL3.onTrue(MoveL3())
//        OI.moveL4.onTrue(MoveL4())
//        OI.moveToIntake.onTrue(MoveToIntake())
//        OI.moveA1.onTrue(MoveA1())
//        OI.moveA2.onTrue(MoveA2())
    }

}