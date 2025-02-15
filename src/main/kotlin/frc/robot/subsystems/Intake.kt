package frc.robot.subsystems

import com.revrobotics.spark.SparkBase
import frc.robot.Constants.IntakeConstants
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.util.Color

import frc.robot.RobotMap
import frc.robot.Constants.IntakeConstants.kv
import frc.robot.Constants.IntakeConstants.ks
import frc.robot.Constants.IntakeConstants.ka
import frc.robot.Constants.IntakeConstants.CORAL_COLOR
import frc.robot.Constants.IntakeConstants.CORAL_COLOR_TOLERANCE
import frc.robot.commands.intake.RunIntake
import kotlin.math.absoluteValue

object Intake : SubsystemBase() {
    val intakeMotor = SparkMax(RobotMap.EndEffectorID, SparkLowLevel.MotorType.kBrushless)
    val IntakeConfig: SparkMaxConfig = SparkMaxConfig()

    var hasCoral = false
    var output = 0.0
    val currentFilter = LinearFilter.movingAverage(20)
    var currentAverage = 0.0

    val buffer = Debouncer(0.1, Debouncer.DebounceType.kRising)
    val bufferTimer = Timer()
    val intakeState get() = bufferTimer.hasElapsed(IntakeConstants.STOP_BUFFER)
    val gracePeriod get() = !bufferTimer.hasElapsed(IntakeConstants.STOP_BUFFER + 5.0)

    // Color sensor values
    val i2cPort = I2C.Port.kOnboard;
    val colorSensor = ColorSensorV3(i2cPort);

    init {
        // Intake motor initialisation stuff
        IntakeConfig
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(20)

        intakeMotor.configure(
            IntakeConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
    }

    override fun periodic() {
        // Coral input motor stuff
        SmartDashboard.putNumber("intake current", intakeMotor.outputCurrent)
        SmartDashboard.putBoolean("has coral", hasCoral)
        SmartDashboard.putNumber("intake output", output)
        SmartDashboard.putNumber("current average", currentAverage)
        SmartDashboard.putNumber("intake timer ", bufferTimer.get())
        currentAverage = currentFilter.calculate(intakeMotor.outputCurrent)

        updateColorSensor() // Also updates hasCoral

    }

    fun runMotor(speed: Double){

        if (!intakeState){
            output = 0.0
            return
        }
        if (buffer.calculate(hasCoral) && !gracePeriod) {
            output = 0.0
            bufferTimer.restart()
            return
        }
        if (gracePeriod && hasCoral) {
            output = speed
        } else {
            output = speed
        }
        intakeMotor.set(output)
    }



    fun outtake() {
        output = -0.4
    }

    fun updateColorSensor(){
        // Get values from the color sensor
        val detectedColor = colorSensor.getColor() // RGB value color sensor sees
        val IR: Double = colorSensor.getIR().toDouble() // Infared light
        val proximity = colorSensor.getProximity().toDouble() // Proximity of the color sensor

        // Display the color sensor values on the SmartDashboard
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR (Not used)", IR);
        SmartDashboard.putNumber("Proximity (Not used)", proximity)

        hasCoral = isCoralInIntake(detectedColor)
    }

    fun isCoralInIntake(detectedColor: Color): Boolean {
//        if ((detectedColor.red - CORAL_COLOR.red).absoluteValue > CORAL_COLOR_TOLERANCE) return false
//        if ((detectedColor.green - CORAL_COLOR.green).absoluteValue > CORAL_COLOR_TOLERANCE) return false
//        if ((detectedColor.blue - CORAL_COLOR.blue).absoluteValue > CORAL_COLOR_TOLERANCE) return false
        if (colorSensor.proximity >= 100.0) {return true}
        return false

        }
    }