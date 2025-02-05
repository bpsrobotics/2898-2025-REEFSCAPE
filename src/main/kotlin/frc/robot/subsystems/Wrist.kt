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
import frc.robot.RobotMap.PivotDriverID


import kotlin.math.cos


object Wrist {
    val pivotMotor = SparkMax(PivotDriverID, SparkLowLevel.MotorType.kBrushless)
    val profileTimer = Timer()
    val pivotencoder = DutyCycleEncoder(PivotPosID)
    var PivotConfig: SparkMaxConfig = SparkMaxConfig()

    var theta = 0.0
    var angVelocity = 0.0
    var angAccel = 0.0
    var armFF = (kG * cos(theta)) + (kS * 0.0) + (kV * angVelocity) + (kA * angAccel)
    var last = getAngle()

    private val botLimit = DigitalInput(RobotMap.LimitBotID)
    private val topLimit = DigitalInput(RobotMap.LimitTopID)

    var targetControl = false
    var targSpeed = 0.0
    private var setpoint = getAngle()
    var outputPower = 0.0
    var rawOutput = 0.0
    val WristPID = PIDController(kP,kI,kD)

    private val constraints = TrapezoidProfile.Constraints(0.0, 0.0)
    val timerThing = Timer()
    var m_profile = TrapezoidProfile(constraints)
    var currentState = TrapezoidProfile.State(radianConversion(), 0.0)
    var goalState = TrapezoidProfile.State(radianConversion(), 0.0)

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
    fun periodic() {
        if (botLimit.get()) {
            resetAngle()
            setpoint = LOWER_LIMIT
        } else if (topLimit.get()) {
            setpoint = UPPER_LIMIT
        }
        motorPeriodic()
    }
    fun getAngle() : Double {
        return pivotencoder.get()
    }
    fun degreeAngle() : Double {
        return ((getAngle() * 360) % 360)
    }
    fun radianConversion() : Double {
        val conversionFactor = (Math.PI / 180)
        return (degreeAngle() * conversionFactor)
    }

    fun motorPeriodic() {
        val curTime = Timer.getFPGATimestamp()
        val dT = curTime - prevUpdateTime
        prevUpdateTime = curTime
        theta = radianConversion() - last
        angVelocity = theta / dT
        angAccel = angVelocity / dT
        if (targetControl) {
            targSpeed = m_profile.calculate(profileTimer.get(), currentState, goalState).velocity
            outputPower = armFF
            outputPower += WristPID.calculate(radianConversion(), goalState.position)
            pivotMotor.setVoltage(outputPower.clamp(NEG_MAX_OUTPUT, POS_MAX_OUTPUT))
        } else {
            currentState.position = getAngle()
            currentState.velocity = 0.0
            pivotMotor.set(rawOutput.clamp(NEG_MAX_OUTPUT, POS_MAX_OUTPUT))
        }
        last = radianConversion()
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

    fun resetAngle(): Double {
        val zeroOffset = pivotencoder.get()
        return zeroOffset  //not sure if this is good so don't use this
    }

    fun setGoal(newPos: Double) { //set limits in the if statement
        if (newPos !in 0.0.. 6.28) return
        setpoint = newPos //newPos should be in radians, CONVERT TO RADIANS!!!!
        profileTimer.reset()
        profileTimer.start()
        currentState = TrapezoidProfile.State(radianConversion(), 0.0)
        goalState = TrapezoidProfile.State(setpoint, 0.0)
    }

}