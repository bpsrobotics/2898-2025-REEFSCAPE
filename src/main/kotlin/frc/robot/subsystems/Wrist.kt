package frc.robot.subsystems

import beaverlib.utils.Units.Angular.rotations
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.MathUtil.angleModulus
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.PivotConstants.LOWER_LIMIT
import frc.robot.Constants.PivotConstants.Max_Accel
import frc.robot.Constants.PivotConstants.Max_Velocity
import frc.robot.Constants.PivotConstants.UPPER_LIMIT
import frc.robot.Constants.PivotConstants.kg
import frc.robot.Constants.PivotConstants.ks
import frc.robot.Constants.PivotConstants.kv
import frc.robot.Constants.PivotConstants.ka
import frc.robot.Constants.PivotConstants.kd
import frc.robot.Constants.PivotConstants.ki
import frc.robot.Constants.PivotConstants.kp
import frc.robot.RobotMap.PivotDriverID
import frc.robot.RobotMap.PivotPosID
import kotlin.math.PI

object Wrist : SubsystemBase() {
    private val armMotor = SparkMax(PivotDriverID, SparkLowLevel.MotorType.kBrushless)
    val encoder = DutyCycleEncoder(PivotPosID, 2.0 * PI, PI)
    private val wristConfig : SparkMaxConfig = SparkMaxConfig()

    var setpoint = getPos()
    var stopped = false

    var voltageApplied = 0.0
    var angVel = 0.0
    var angAccel = 0.0
    private val constraints = TrapezoidProfile.Constraints(Max_Velocity, Max_Accel)
    val profile = TrapezoidProfile(constraints)
    val profileTimer = Timer()

    var curState = TrapezoidProfile.State(getPos(), 0.0)
    var goalState = TrapezoidProfile.State(getPos(), 0.0)
    var neededVoltage = 0.0

    val wristFeedForward = ArmFeedforward(ks, kg, kv, ka)
    val wristPID = PIDController(kp, ki, kd)



    fun getPos(): Double {
        val p = encoder.get().rotations
        return p.asRadians
    }

    init {
        wristConfig
            .smartCurrentLimit(40)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
        armMotor.configure(wristConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)


    }

    override fun periodic() {
    //todo configure this but we probably can put it in the feedbackcontroller function
    }

    fun feedbackController (desiredAngVel: Double) {
        neededVoltage = wristFeedForward.calculate(getPos(), desiredAngVel) + wristPID.calculate(getPos(), goalState.position)
        //todo configure what happens when wrist reaches top (Pi/2) and bottom limits (3PI/2 or -PI/2)
    }

    fun setGoal(newPos: Double) {
        if (newPos !in LOWER_LIMIT..UPPER_LIMIT) return
        setpoint = newPos
        profileTimer.reset()
        profileTimer.start()
        curState = TrapezoidProfile.State(getPos(), angVel)
        goalState = TrapezoidProfile.State(setpoint, 0.0)
    }

    fun resetPos() {
        //todo configure a reset function if we need it.
    }
}