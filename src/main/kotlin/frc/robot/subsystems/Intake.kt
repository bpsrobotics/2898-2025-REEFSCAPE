package frc.robot.subsystems

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.event.BooleanEvent
import edu.wpi.first.wpilibj.event.EventLoop
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.IntakeConstants
import frc.robot.RobotMap
import frc.robot.commands.intake.StopIntake

object Intake : SubsystemBase() {
    private val intakeMotor = SparkMax(RobotMap.EndEffectorID, SparkLowLevel.MotorType.kBrushless)
    private val IntakeConfig: SparkMaxConfig = SparkMaxConfig()

    private val loop = EventLoop()
    private val sensorPort = DigitalInput(IntakeConstants.SENSOR_PIN)
    private val sensorEv = BooleanEvent(loop) { !sensorPort.get() }
    val hasCoral: BooleanEvent = sensorEv.debounce(IntakeConstants.DEBOUNCE, Debouncer.DebounceType.kRising)

    enum class RollerStates(val speed: Double) {
        CoralIdle(-0.1),
        AlgaeIdle(-0.4),
        
    }


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
        defaultCommand = StopIntake()
    }

    override fun periodic() {
        loop.poll()

        SmartDashboard.putBoolean("sensor port HIGH", sensorPort.get())
        SmartDashboard.putBoolean("has coral", hasCoral.asBoolean)
    }

    fun runMotor(speed: Double) {
        intakeMotor.set(speed)
    }

    fun stop() {
        intakeMotor.stopMotor()
    }
}