package frc.robot.subsystems

import com.revrobotics.spark.SparkBase
import frc.robot.Constants.IntakeConstants
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import frc.robot.RobotMap
import frc.robot.Constants.IntakeConstants.kv
import frc.robot.Constants.IntakeConstants.ks
import frc.robot.Constants.IntakeConstants.ka


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
        SmartDashboard.putNumber("intake current", intakeMotor.outputCurrent)
        SmartDashboard.putBoolean("has coral", hasCoral)
        SmartDashboard.putNumber("intake output", output)
        SmartDashboard.putNumber("current average", currentAverage)
        SmartDashboard.putNumber("intake timer ", bufferTimer.get())
        currentAverage = currentFilter.calculate(intakeMotor.outputCurrent)

        intakeMotor.set(output)
    }

    fun intake(speed: Double){
        if (intakeState) {
            if (buffer.calculate(currentAverage > IntakeConstants.CURRENT_WHEN_ROBOT_HAS_CORAL) && !hasCoral && !gracePeriod) {
                output = 0.0
                bufferTimer.reset()
                bufferTimer.start()
                hasCoral = true
            } else {
                if (gracePeriod) {
                    output = speed
                } else {
                    output = speed
                    hasCoral = false
                }
            }
        } else {
            println("stopping intake")
            output = 0.0
        }
    }
    fun ffController (goalVelocity: Double) {
        voltage = flywheelFF.calculate(goalVelocity)
    }

    fun outtake() {
        output = -0.4
    }
}