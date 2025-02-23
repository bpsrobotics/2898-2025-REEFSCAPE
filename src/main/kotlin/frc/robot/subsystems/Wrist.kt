package frc.robot.subsystems

import beaverlib.utils.Sugar.clamp
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.PivotConstants.Max_Accel
import frc.robot.Constants.PivotConstants.Max_Velocity
import frc.robot.Constants.PivotConstants.NEG_MAX_OUTPUT
import frc.robot.Constants.PivotConstants.ObstructionAngle
import frc.robot.Constants.PivotConstants.POS_MAX_OUTPUT
import frc.robot.Constants.PivotConstants.kD
import frc.robot.Constants.PivotConstants.kG
import frc.robot.Constants.PivotConstants.kI
import frc.robot.Constants.PivotConstants.kP
import frc.robot.Constants.PivotConstants.kS
import frc.robot.Constants.PivotConstants.kV
import frc.robot.RobotMap.PivotDriverID
import frc.robot.RobotMap.PivotPosID
import frc.robot.commands.wrist.StabilizeWrist
import kotlin.math.PI

object Wrist : SubsystemBase() {
    val armMotor = SparkMax(PivotDriverID, SparkLowLevel.MotorType.kBrushless)
    val encoder = DutyCycleEncoder(PivotPosID)
    private val wristConfig : SparkMaxConfig = SparkMaxConfig()

    var setpoint = getPos()

    var velocity = 0.0
    private val constraints = TrapezoidProfile.Constraints(Max_Velocity, Max_Accel)
    val encoderOffset = 0.0
    val profile = TrapezoidProfile(constraints)

    var deltaAngle = 0.0
    var lastPosition = getPos()

    var currentState = TrapezoidProfile.State(getPos(), 0.0)
    var goalState = TrapezoidProfile.State(getPos(), 0.0)

    val feedForward = ArmFeedforward(kS, kG, kV)
    val pid = PIDController(kP, kI, kD)



    init {
        wristConfig
            .smartCurrentLimit(40)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
        armMotor.configure(wristConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)

        encoder.setDutyCycleRange(0.0, 2.0 * PI)
        defaultCommand = StabilizeWrist()
    }

    override fun periodic() {
        deltaAngle = getPos() - lastPosition
        lastPosition = getPos()
    }

    fun getPos(): Double {
        val p = encoder.get() + encoderOffset
        return p
    }

    fun closedLoopControl(targetSpeed: Double) {
        val outputPower = feedForward.calculate(getPos(), targetSpeed) + pid.calculate(getPos(), goalState.position)
        armMotor.setVoltage(outputPower.clamp(NEG_MAX_OUTPUT, POS_MAX_OUTPUT))
    }

    fun isObstructing() : Boolean {
        return getPos() < ObstructionAngle
    }

    fun setVoltage(voltage: Double) {
        armMotor.setVoltage(voltage)
    }


}