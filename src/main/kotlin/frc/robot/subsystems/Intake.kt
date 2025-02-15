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
import kotlin.math.absoluteValue

object Intake : SubsystemBase() {
    val intakeMotor = SparkMax(RobotMap.IntakeID, SparkLowLevel.MotorType.kBrushless)
    val IntakeConfig: SparkMaxConfig = SparkMaxConfig()

    var voltage = 0.0
    val flywheelFF = SimpleMotorFeedforward(ks, kv, ka)

    var hasCoral = false
    var output = 0.0
    val currentFilter = LinearFilter.movingAverage(20)
    var currentAverage = 0.0

    val buffer = Debouncer(0.04, Debouncer.DebounceType.kRising)
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

        intakeMotor.set(output)
    }

    fun intake(speed: Double){
        updateColorSensor() // Also updates hasCoral

        if (intakeState) {
            if (buffer.calculate(hasCoral) && !gracePeriod) {
                output = 0.0
                bufferTimer.reset()
                bufferTimer.start()
            } else {
                if (gracePeriod) {
                    output = speed
                } else {
                    output = speed
                }
            }
        } else {
            println("stopping intake")
            output = 0.0
        }
    }
    fun algaeIntake(speed: Double) {
        IntakeConfig
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(20)
            .inverted(true)

        intakeMotor.configure(
            IntakeConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        output = speed //fixme; make sure this doesn't break the code
    }
    fun ffController (goalVelocity: Double) {
        voltage = flywheelFF.calculate(goalVelocity)
    }

    fun outtake() {
        output = -0.4
    }

    fun updateColorSensor(){
        // Get values from the color sensor
        val detectedColor = colorSensor.getColor() // RGB value color sensor sees
        val IR: Double = colorSensor.getIR().toDouble() // Infared light
        val proximity = colorSensor.getProximity().toDouble() // Proximity of the color sensor

        val tooClose = 0.0 //todo actually check and configure this

        // Display the color sensor values on the SmartDashboard
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR (Not used)", IR);
        SmartDashboard.putNumber("Proximity (Not used)", proximity)

        if (proximity > tooClose) {
            hasCoral = false
        }
        else {
            hasCoral = false
        }

        //hasCoral = isCoralInIntake(detectedColor)
    }

    fun isCoralInIntake(detectedColor: Color): Boolean {
        if ((detectedColor.red - CORAL_COLOR.red).absoluteValue > CORAL_COLOR_TOLERANCE) return false
        if ((detectedColor.green - CORAL_COLOR.green).absoluteValue > CORAL_COLOR_TOLERANCE) return false
        if ((detectedColor.blue - CORAL_COLOR.blue).absoluteValue > CORAL_COLOR_TOLERANCE) return false
        return true
        }
    } //todo: figure out how to make so that we can detect coral w/ no color sensor