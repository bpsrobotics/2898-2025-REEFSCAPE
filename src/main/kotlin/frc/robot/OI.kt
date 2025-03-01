package frc.robot

import beaverlib.utils.geometry.Vector2

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.POVButton
import frc.beaverlib.async.Promise
import frc.robot.Constants.ButtonConstants.ALGAE_B1
import frc.robot.Constants.ButtonConstants.ALGAE_B2
import frc.robot.Constants.ButtonConstants.BASE_STAGE
import frc.robot.Constants.ButtonConstants.RESET_GYRO
import frc.robot.Constants.ButtonConstants.CORAL_L2
import frc.robot.Constants.ButtonConstants.CORAL_L3
import frc.robot.Constants.ButtonConstants.CORAL_L4

import kotlin.math.pow
import kotlin.math.sign

/**
 * The Operating Interface object.
 * This is where you put joystick, button, or keyboard inputs.
 *
 * A note about delegated properties, which are used in this object:
 *  A delegated property is where getting (or setting) a field is outsourced
 *  to another object.  Then, whenever you read the property, it asks the
 *  object the property is delegated to for the value.
 */
@Suppress("unused")
object OI : SubsystemBase() {
    /**
     * Threshold below which [process] will return 0.
     * 0.1 historically used, but optimal value unknown.
     */
    private const val DEADZONE_THRESHOLD = 0.1

    /**
     * Utility function for controller axis, optional deadzone and square/cube for extra fine-grain control
     */
    private fun process(
        input: Double,
        deadzone: Boolean = false,
        square: Boolean = false,
        cube: Boolean = false
    ): Double {
        var output = 0.0

        if (deadzone) {
            output = MathUtil.applyDeadband(input, DEADZONE_THRESHOLD)
        }

        if (square) {
            // To keep the signage for output, we multiply by sign(output). This keeps negative inputs resulting in negative outputs.
            output = output.pow(2) * sign(output)
        }

        if (cube) {
            // Because cubing is an odd number of multiplications, we don't need to multiply by sign(output) here.
            output = output.pow(3)
        }

        return output
    }

    // conflicts with the other definition, name it something else after compilation
    @JvmName("process1")
    fun Double.process(deadzone: Boolean = false, square: Boolean = false, cube: Boolean = false) =
        process(this, deadzone, square, cube)

    private val driverController = CommandXboxController(0)
    private val operatorController = CommandJoystick(1)

    // Right joystick y-axis.  Controller mapping can be tricky, the best way is to use the driver station to see what buttons and axis are being pressed.
    // Squared for better control on turn, cubed on throttle
    /** Driver controller's throttle on the left joystick for the X Axis, from -1 (left) to 1 (right) */
    val translationX
        get() = process(driverController.leftX, deadzone = true, square = false)

    /** Driver controller's throttle on the left joystick for the Y Axis, from -1 (down) to 1 (up) */
    val translationY
        get() = process(driverController.leftY, deadzone = true, square = false)

    /** Driver controller's throttle on the right joystick for the X Axis, from -1 (left) to 1 (right) */
    val turnX
        get() = process(driverController.rightX, deadzone = true, square = false)
    /** Driver controller's throttle on the right joystick for the Y Axis, from -1 (down) to 1 (up) */
    val turnY
        get() = process(driverController.rightY, deadzone = true, square = false)
    val leftTrigger
        get() = driverController.leftTriggerAxis
    val rightTrigger
        get() = driverController.rightTriggerAxis

    // Coral out take positions move to
    val moveL1 = operatorController.button(BASE_STAGE)
    val moveL2 = operatorController.button(CORAL_L2)
    val moveL3 = operatorController.button(CORAL_L3)
    val moveL4 = operatorController.button(CORAL_L4)
    val moveA1 = operatorController.button(ALGAE_B1)
    val moveA2 = operatorController.button(ALGAE_B2)

    val resetGyro = driverController.rightBumper()
    val sysidFQ = driverController.x()
    val sysidBQ = driverController.y()
    val sysidFD = driverController.b()
    val sysidBD = driverController.a()




    val highHatForward = operatorController.pov(0)
    val highHatBack = operatorController.pov(180)
//    val hatVector get() = when (operatorController.pov) {
//        0 -> Vector2(0.0,1.0)
//        90 -> Vector2(1.0,0.0)
//        180 -> Vector2(0.0,-1.0)
//        270 -> Vector2(-1.0,0.0)
//        else -> Vector2.zero()
//    }

    val intakeSpeed get() = operatorController.throttle

//    val runIntake: BooleanEvent = BooleanEvent(loop) { hatVector == Vector(0,-1) }
//    val shooterOutake: BooleanEvent = BooleanEvent(loop) { hatVector == Vector(0, 1) }



    enum class Direction {
        LEFT, RIGHT, UP, DOWN, UPLEFT, UPRIGHT, DOWNLEFT, DOWNRIGHT, INACTIVE;

        fun mirrored() = when (this) {
            LEFT  -> RIGHT
            RIGHT -> LEFT
            else  -> this
        }
        fun toVector() = when(this) {
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


    //    val operatorTrigger: BooleanEvent = operatorController.button(1, loop)
    //val operatorTrigger get() = operatorController.getRawButton(1)
    //    val operatorTriggerReleased: BooleanEvent = operatorTrigger.falling()
    object Rumble {
        private var isRumbling  = false
        private var rumbleTime  = 0.0
        private val rumblePower = 0.0
        private val rumbleSide = GenericHID.RumbleType.kRightRumble
        private val rumbleTimer = Timer()
        private val waiting = mutableSetOf<Promise<Unit>>()
        fun set(time: Double, power: Double, side: GenericHID.RumbleType = GenericHID.RumbleType.kBothRumble){
            rumbleTimer.reset()
            rumbleTime = time
            driverController.setRumble(side, power)
            rumbleTimer.start()
        }
        fun until(promise: Promise<Unit>, power: Double = 1.0, side: GenericHID.RumbleType = GenericHID.RumbleType.kBothRumble) {
            driverController.setRumble(side, power)
            waiting.add(promise)
            promise.then { update(); Promise.resolve(Unit) }
        }
        fun update(){
            if(rumbleTimer.hasElapsed(rumbleTime) && !(try {
                    waiting.map { it.hasFulfilled }.reduce { acc, b -> acc or b }} catch (_:Throwable) {false})) {
                driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
            }
            waiting.forEach {
                if (it.hasFulfilled) waiting.remove(it)
            }
        }
    }

    override fun periodic(){
        Rumble.update()
    }
}