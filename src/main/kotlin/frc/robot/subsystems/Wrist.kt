package frc.robot.subsystems

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.MathUtil.angleModulus
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.PivotConstants.LOWER_LIMIT
import frc.robot.Constants.PivotConstants.Max_Accel
import frc.robot.Constants.PivotConstants.Max_Velocity
import frc.robot.Constants.PivotConstants.UPPER_LIMIT
import frc.robot.RobotMap.PivotDriverID
import frc.robot.RobotMap.PivotPosID
import kotlin.math.PI

object Wrist : SubsystemBase() {
    private val armMotor = SparkMax(PivotDriverID, SparkLowLevel.MotorType.kBrushless)
    private val encoder = DutyCycleEncoder(PivotPosID, 2.0 * PI, PI)
    private val wristConfig : SparkMaxConfig = SparkMaxConfig()

    var setpoint = getPos()
    var stopped = false

    val ksin = 0.0
    val ks = 0.0
    val kv = 0.0

    var voltageApplied = 0.0
    var vel = 0.0
    private val constraints = TrapezoidProfile.Constraints(Max_Velocity, Max_Accel)
    val profile = TrapezoidProfile(constraints)
    val profileTimer = Timer()

    var curState = TrapezoidProfile.State(getPos(), 0.0)
    var goalState = TrapezoidProfile.State(getPos(), 0.0)



    fun getPos(): Double {
        val p = encoder.get()
        return angleModulus(p)
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

    }

    fun motorPeriodic() {

    }

    fun setGoal(newPos: Double) {
        if (newPos !in LOWER_LIMIT..UPPER_LIMIT) return
        setpoint = newPos
        profileTimer.reset()
        profileTimer.start()
        curState = TrapezoidProfile.State(getPos(), vel)
        goalState = TrapezoidProfile.State(setpoint, 0.0)
    }




}