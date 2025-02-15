// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

//import com.team2898.robot.Constants.OperatorConstants

import beaverlib.utils.geometry.Vector2
import com.pathplanner.lib.auto.AutoBuilder

import frc.robot.OI.intakeSpeed
import frc.robot.OI.operatorTrigger
import frc.robot.OI.resetGyro
import frc.robot.OI.rightTrigger
import frc.robot.OI.translationX
import frc.robot.OI.translationY
import frc.robot.OI.turnX
import frc.robot.subsystems.Drivetrain.getDriveSysIDCommand
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.beaverlib.async.Promise
import frc.robot.OI.process
import frc.robot.commands.swerve.*
import frc.robot.subsystems.Drivetrain
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


    private fun process(
        input: Double,
        deadzone: Boolean = false,
        square: Boolean = false,
        cube: Boolean = false
    ): Double {
        var output = 0.0

        if (deadzone) output = MathUtil.applyDeadband(input, Constants.OIConstants.DEADZONE_THRESHOLD)

        // To keep the signage for output, we multiply by sign(output). This keeps negative inputs resulting in negative outputs.
        if (square) output = output.pow(2) * sign(output)

        // Because cubing is an odd number of multiplications, we don't need to multiply by sign(output) here.
        if (cube) output = output.pow(3)

        return output
    }

    // conflicts with the other definition, name it something else after compilation
    @JvmName("process1")
    fun Double.process(deadzone: Boolean = false, square: Boolean = false, cube: Boolean = false) =
        process(this, deadzone, square, cube)

    private val driverController = XboxController(0)
    private val commandDriverController = CommandXboxController(0)
    class Rumble(val controller : CommandXboxController, val time: Double = 1.0, val rumblePower : Double = 1.0, val rumbleSide : GenericHID.RumbleType = GenericHID.RumbleType.kRightRumble ) : Command() {
        val timer = Timer()
        override fun initialize() { timer.restart() }
        override fun execute() { controller.setRumble(rumbleSide, rumblePower) }

        override fun end(interrupted: Boolean) { controller.setRumble(rumbleSide, 0.0) }

        override fun isFinished(): Boolean { return timer.hasElapsed(time) }
    }


    private var autoCommandChooser: SendableChooser<Command> = SendableChooser()
    private val operatorController = Joystick(1)

    val quickTurnRight
        get() = process(commandDriverController.rightTriggerAxis, deadzone = true, square = true)
    val quickTurnLeft
        get() = process(commandDriverController.leftTriggerAxis, deadzone = true, square = true)

    /** Driver controller's throttle on the left joystick for the X Axis, from -1 (left) to 1 (right) */
    val translationX
        get() = process(commandDriverController.leftX, deadzone = true, square = false)

    /** Driver controller's throttle on the left joystick for the Y Axis, from -1 (down) to 1 (up) */
    val translationY
        get() = process(commandDriverController.leftY, deadzone = true, square = false)

    /** Driver controller's throttle on the right joystick for the X Axis, from -1 (left) to 1 (right) */
    val turnX
        get() = process(commandDriverController.rightX, deadzone = true, square = false)
    /** Driver controller's throttle on the right joystick for the Y Axis, from -1 (down) to 1 (up) */
    val turnY
        get() = process(commandDriverController.rightY, deadzone = true, square = false)

    val leftTrigger
        get() = commandDriverController.leftTriggerAxis
    val rightTrigger
        get() = commandDriverController.rightTriggerAxis

    val driverY = JoystickButton(driverController, 4)
    val driverX = JoystickButton(driverController, 3)
    val coralAlign = JoystickButton(driverController, 5)
    val resetGyro= JoystickButton(driverController, 6)
    val climb = JoystickButton(operatorController, 12)

    val highHat get() = operatorController.pov
    val hatVector get() = when (operatorController.pov) {
        0 -> Vector2(0.0,1.0)
        90 -> Vector2(1.0,0.0)
        180 -> Vector2(0.0,-1.0)
        270 -> Vector2(-1.0,0.0)
        else -> Vector2.zero()
    }

    val armSelectUp = JoystickButton(operatorController, 5)
    val armSelectDown = JoystickButton(operatorController, 3)
    val armDirectGround = JoystickButton(operatorController, 11)
    val armDirectStowed = JoystickButton(operatorController, 8)
    val armDirectAmp = JoystickButton(operatorController, 7)
    val armDirectShooter1 = JoystickButton(operatorController, 9)
    val armDirectShooter2 = JoystickButton(operatorController, 10)
    val climbUp = JoystickButton(operatorController, 6)
    val climbDown = JoystickButton(operatorController, 4)

    enum class Direction {
        LEFT, RIGHT, UP, DOWN, UPLEFT, UPRIGHT, DOWNLEFT, DOWNRIGHT, INACTIVE;

        fun mirrored() = when (this) {
            LEFT  -> RIGHT
            RIGHT -> LEFT
            else  -> this
        }
        fun toVector() = when (this) {
            LEFT -> Vector2(-1.0,0.0)
            RIGHT -> Vector2(1.0,0.0)
            UP -> Vector2(0.0,1.0)
            DOWN -> Vector2(0.0,-1.0)
            INACTIVE -> Vector2.zero()
            UPLEFT -> Vector2(-1.0,1.0)
            UPRIGHT -> Vector2(1.0, 1.0)
            DOWNLEFT -> Vector2(-1.0, -1.0)
            DOWNRIGHT -> Vector2(1.0, -1.0)
        }
    }

    val alignmentPad get() = when(driverController.pov) {
        0    -> frc.robot.OI.Direction.UP
        45   -> frc.robot.OI.Direction.UPRIGHT
        90   -> frc.robot.OI.Direction.RIGHT
        135  -> frc.robot.OI.Direction.DOWNRIGHT
        180  -> frc.robot.OI.Direction.DOWN
        225  -> frc.robot.OI.Direction.DOWNLEFT
        270  -> frc.robot.OI.Direction.LEFT
        315  -> frc.robot.OI.Direction.UPLEFT
        else -> frc.robot.OI.Direction.INACTIVE
    }

    val operatorTrigger = JoystickButton(operatorController, 1)




    val teleopDrive: TeleopDriveCommand =
        TeleopDriveCommand(
            { MathUtil.applyDeadband(translationY, 0.1) },
            { MathUtil.applyDeadband(translationX, 0.1) },
            { MathUtil.applyDeadband(-turnX, 0.1)},
            { true },
            { false }
        )

    val navXResetCommand: NavXReset = NavXReset()

    val intakeSpeed get() = operatorController.throttle



    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        initializeObjects()

        // Configure the trigger bindings

        //autoCommandChooser = AutoBuilder.buildAutoChooser("6piece")

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
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [Trigger.Trigger] constructor with an arbitrary
     * predicate, or via the named factories in [ ]'s subclasses for [ ]/[ PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller] controllers or [Flight][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        coralAlign.whileTrue(CoralAlignCommandWithOdometry(teleopDrive.speedConsumer))


        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        //Trigger { m_exampleSubsystem.exampleCondition() }
        //        .onTrue(ExampleCommand(m_exampleSubsystem))

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
        //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand())



        resetGyro.whileTrue(navXResetCommand)

    }

}