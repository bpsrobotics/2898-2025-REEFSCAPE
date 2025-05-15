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
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.OI.autoIntake
import frc.robot.OI.elevBWStepper
import frc.robot.OI.elevFWStepper
import frc.robot.OI.highHatBack
import frc.robot.OI.highHatForward
import frc.robot.OI.pivotBWStepper
import frc.robot.OI.pivotFWStepper
import frc.robot.OI.resetGyro
import frc.robot.OI.toggleWrist
import frc.robot.commands.elevator.*
import frc.robot.commands.intake.RunIntake
import frc.robot.commands.intake.RunOuttake
import frc.robot.commands.sequence.*
import frc.robot.commands.swerve.NavXReset
import frc.robot.commands.swerve.TeleopDriveCommand
import frc.robot.commands.swerve.*
import frc.robot.commands.wrist.*
import frc.robot.subsystems.*
import kotlin.math.pow
import kotlin.math.sign

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
    val alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)

    val reverseDrive = if(alliance == DriverStation.Alliance.Red) {-1.0} else {1.0}

    val teleopDrive: TeleopDriveCommand =
        TeleopDriveCommand(
            { MathUtil.applyDeadband(translationY*reverseDrive, 0.1) },
            { MathUtil.applyDeadband(translationX*reverseDrive, 0.1) },
            { MathUtil.applyDeadband(-turnX, 0.1)},
            { true },
            { false }
        )


    val navXResetCommand: NavXReset = NavXReset()



    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {


        NamedCommands.registerCommand("coralouttake", RunOuttake(0.8)) //Todo set this properly
        //todo NamedCommands.registerCommand("algaeintake", AlgaeIntakeOutake())
        NamedCommands.registerCommand("L1", Stow())
        NamedCommands.registerCommand("L2", PositionL2())
        NamedCommands.registerCommand("L3", MoveElevator(Constants.ElevatorConstants.ElevatorState.L3.position))
        NamedCommands.registerCommand("L4", SequentialCommandGroup(MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
            MoveElevator(Constants.ElevatorConstants.ElevatorState.L4.position),
            MoveWrist(Constants.PivotConstants.PivotState.VerticalBranch.position)
        ))
        NamedCommands.registerCommand("PlaceSequence", SequentialCommandGroup(MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
            MoveElevator(Constants.ElevatorConstants.ElevatorState.L4.position),
            MoveWrist(Constants.PivotConstants.PivotState.VerticalBranch.position),
            RunOuttake(0.8, 0.5)
        ))
        NamedCommands.registerCommand("autointake", RunIntake())
        NamedCommands.registerCommand("GetCoralStationPiece", SequentialCommandGroup(MoveElevator(Constants.ElevatorConstants.ElevatorState.Stow.position),
            MoveWrist(Constants.PivotConstants.PivotState.Stow.position),
            RunIntake()
        ))
        NamedCommands.registerCommand("stabilize", StabilizeElevator())
        initializeObjects()
        // Configure the trigger bindings

        autoCommandChooser = AutoBuilder.buildAutoChooser("4-Piece-Low")

        Drivetrain.defaultCommand = teleopDrive

        configureBindings()

        SmartDashboard.putData("Auto mode", autoCommandChooser)


    }
    fun getAutonomousCommand(): Command{
        val path = autoCommandChooser.selected
        return path
    }

    private fun initializeObjects() {
        Drivetrain
        PathPlanner
        Wrist
        Elevator
        Intake
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [Trigger.Trigger] constructor with an arbitrary
     * predicate, or via the named factories in [ ]'s subclasses for [ ]/[ PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller] controllers or [Flight][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
//        OI.coralAlignLeft.whileTrue(ReefAlignCommand(teleopDrive.speedConsumer, Constants.VisionConstants.CORAL_OFFSET_FROM_CENTER))
//        OI.coralAlignRight.whileTrue(ReefAlignCommand(teleopDrive.speedConsumer, -Constants.VisionConstants.CORAL_OFFSET_FROM_CENTER))


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

        toggleWrist.onTrue(ToggleState())

        autoIntake.onTrue(RunIntake())

        highHatForward.whileTrue(RunOuttake(0.5))
        highHatBack.whileTrue(RunOuttake(-1.0))

        elevFWStepper.onTrue(MoveElevatorBy( 0.05 ))
        elevBWStepper.onTrue(MoveElevatorBy(-0.05))

//        pivotFWStepper.whileTrue(VoltageWrist(0.2))
            pivotFWStepper.onTrue(MoveWristBy(-0.4))
        pivotBWStepper.onTrue(MoveWristBy(0.4))

//        pivotBWStepper.whileTrue(VoltageWrist(-0.2))


        OI.moveA1.onTrue(
            SequentialCommandGroup(
            MoveElevator(Constants.ElevatorConstants.ElevatorState.A1.position),
            MoveWrist(Constants.PivotConstants.PivotState.Algae.position)

        )
        )
        OI.moveA2.onTrue(      SequentialCommandGroup(
            MoveElevator(Constants.ElevatorConstants.ElevatorState.A2.position),
            MoveWrist(Constants.PivotConstants.PivotState.Algae.position)

        ))
        OI.moveL1.onTrue(
            MoveElevator(Constants.ElevatorConstants.ElevatorState.Stow.position)
        )
        OI.moveL2.onTrue(  SequentialCommandGroup(MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
            MoveElevator(Constants.ElevatorConstants.ElevatorState.L2.position),
            MoveWrist(Constants.PivotConstants.PivotState.AngleBranch.position)

        ))
        OI.moveL3.onTrue(        SequentialCommandGroup(MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
            MoveElevator(Constants.ElevatorConstants.ElevatorState.L3.position),
            MoveWrist(Constants.PivotConstants.PivotState.AngleBranch.position)
        ))
        OI.moveL4.onTrue(SequentialCommandGroup(MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
            MoveElevator(Constants.ElevatorConstants.ElevatorState.L4.position),
            MoveWrist(Constants.PivotConstants.PivotState.VerticalBranch.position)
        ))

//        OI.moveL1.onTrue(MoveElevator(Constants.ElevatorConstants.ElevatorState.Stow.position))
//        OI.moveL2.onTrue(MoveElevator(Constants.ElevatorConstants.ElevatorState.L2.position))
//        OI.moveL3.onTrue(MoveElevator(Constants.ElevatorConstants.ElevatorState.L3.position))




    }

}