package frc.robot.subsystems

import beaverlib.utils.Sugar.clamp
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import com.revrobotics.AbsoluteEncoder
import frc.robot.Constants
import frc.robot.RobotMap
import frc.robot.RobotMap.PivotPosID
import frc.robot.Constants.ElevatorConstants.kA
import frc.robot.Constants.ElevatorConstants.kD
import frc.robot.Constants.ElevatorConstants.kG
import frc.robot.Constants.ElevatorConstants.kI
import frc.robot.Constants.ElevatorConstants.kP
import frc.robot.Constants.ElevatorConstants.kS
import frc.robot.Constants.ElevatorConstants.kV //make new constants for Wrist
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder

import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.Timer
import frc.robot.Constants.ElevatorConstants.LOWER_LIMIT
import frc.robot.Constants.ElevatorConstants.NEG_MAX_OUTPUT
import frc.robot.Constants.ElevatorConstants.POS_MAX_OUTPUT
import frc.robot.Constants.ElevatorConstants.UPPER_LIMIT


import kotlin.math.cos


object Wrist {
    val pivotMotor = SparkMax(PivotDriverID, SparkLowLevel.MotorType.kBrushless)
    val profileTimer = Timer()
    val pivotencoder = DutyCycleEncoder(RobotMap.PivotPosID)
    var PivotConfig: SparkMaxConfig = SparkMaxConfig()

    var theta = 0.0
    var angVelocity = 0.0
    var angAccel = 0.0
    var armFF = (kG * cos(theta)) + (kS * 0.0) + (kV * angVelocity) + (kA * angAccel)
    var last = getAngle()

    private val botLimit = DigitalInput(LimitBotID)
    private val topLimit = DigitalInput(LimitTopID)

    var targetControl = false
    var targSpeed = 0.0
    private var setpoint = getAngle()
    var outputPower = 0.0
    var rawOutput = 0.0
    val WristPID = PIDController(kP,kI,kD)

    private val constraints = TrapezoidProfile.Constraints(0.0, 0.0)
    val timerThing = Timer()
    var m_profile = TrapezoidProfile(constraints)
    var currentState = TrapezoidProfile.State(pivotencoder.get(), 0.0)
    var goalState = TrapezoidProfile.State(pivotencoder.get(), 0.0)

    var prevUpdateTime = 0.0





    init {
        PivotConfig
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(40)

        pivotMotor.configure(
            PivotConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
    }
    override fun periodic() {
        if (botLimit.get()) {
            resetPos()
            setpoint = LOWER_LIMIT
        } else if (topLimit.get()) {
            setpoint = UPPER_LIMIT
        }
        motorPeriodic()
    }
    fun getAngle() : Double {
        return pivotencoder.get()
    }

    fun motorPeriodic() {
        val curTime = Timer.getFPGATimestamp()
        val dT = curTime - prevUpdateTime
        prevUpdateTime = curTime
        theta = pivotencoder.getAngle() - last
        angVelocity = theta / dT
        angAccel = angVelocity / dT
        if (targetControl) {
            targSpeed = m_profile.calculate(profileTimer.get(), currentState, goalState).velocity
            outputPower = armFF
            outputPower += WristPID.calculate(pivotencoder.get(), goalState.position)
            pivotMotor.setVoltage(outputPower.clamp(NEG_MAX_OUTPUT, POS_MAX_OUTPUT))
        } else {
            currentState.position = getAngle()
            currentState.velocity = 0.0
            pivotMotor.set(rawOutput.clamp(NEG_MAX_OUTPUT, POS_MAX_OUTPUT))
        }
        last = getAngle()
    }

    fun voltMore(output : Double) {
        rawOutput = output
    }
    fun release() {
        PivotConfig.idleMode(
            SparkBaseConfig.IdleMode.kCoast
        )
        pivotMotor.setVoltage(0.0)
    }

    fun resetPos() {
        return pivotencoder.reset()
    }

}