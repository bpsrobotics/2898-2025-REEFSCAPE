package frc.robot.subsystems

import beaverlib.utils.Sugar.clamp
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.MathUtil.angleModulus
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import frc.robot.Constants
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
private val wristConfig : SparkMaxConfig = SparkMaxConfig()

object Wrist : SubsystemBase() {
    val armMotor = SparkMax(PivotDriverID, SparkLowLevel.MotorType.kBrushless)
    val encoder = DutyCycleEncoder(PivotPosID,2 * PI,3.9)

    var velocity = 0.0
    private val constraints = TrapezoidProfile.Constraints(Max_Velocity, Max_Accel)

    var deltaAngle = 0.0
    val rateTimer = Timer()
    var rate = 0.0
    var lastPosition = pos


    val feedForward = ArmFeedforward(kS, kG, kV)
    val pid = PIDController(kP, kI, kD)
    val profiledPID = ProfiledPIDController(kP, kI, kD, constraints)

    val pos get() = angleModulus(encoder.get())

    var coralState = true
    var presetGoal = Constants.PivotConstants.PivotState.Stow

    var prev_target = pos

    init {
        // Wrist Motor Configuration
        wristConfig
            .smartCurrentLimit(40)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
        armMotor.configure(wristConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)

        SmartDashboard.putNumber("/wrist/pos", pos)
        SmartDashboard.putNumber("/wrist/rate", rate)
        SmartDashboard.putBoolean("coral state", coralState)
        SmartDashboard.putNumber("/wrist/errorspeed", profiledPID.velocityError)


        encoder.setInverted(true)
        defaultCommand = StabilizeWrist()
        rateTimer.restart()

    }

    override fun periodic() {

        deltaAngle = pos - lastPosition
        rate = deltaAngle / rateTimer.get()
        lastPosition = pos
        rateTimer.restart()
        SmartDashboard.putNumber("/wrist/delta_angle", deltaAngle)
        SmartDashboard.putNumber("/wrist/rate", rate)
        SmartDashboard.putNumber("/wrist/pos", pos)
        SmartDashboard.putNumber("/wrist/errorspeed", profiledPID.velocityError)


    }


    fun setVoltage(voltage: Double) {
        armMotor.setVoltage(voltage)
    }
    fun set(speed: Double) {
        armMotor.set(speed)
    }

    fun closedLoopPositionControl(targetSpeed: Double) {
        val outputPower = feedForward.calculate(pos, 0.0) + pid.calculate(pos, profiledPID.goal.position)
        SmartDashboard.putNumber("/wrist/output_power", outputPower)
        armMotor.setVoltage(outputPower.clamp(NEG_MAX_OUTPUT, POS_MAX_OUTPUT))
    }

    fun profiledPIDControl(targetPos: Double) {
        profiledPID.setGoal(targetPos)
        val pidOutput = profiledPID.calculate(pos, targetPos)
        val ffOutput = feedForward.calculate(pos, profiledPID.setpoint.velocity)
        val outputPower = pidOutput + ffOutput
        SmartDashboard.putNumber("/wrist/ff_o", ffOutput)
        SmartDashboard.putNumber("/wrist/pid_o", pidOutput)
        SmartDashboard.putNumber("/wrist/targ_Speed", profiledPID.setpoint.velocity)
        SmartDashboard.putNumber("/wrist/output_power", outputPower)
        armMotor.setVoltage(outputPower.clamp(NEG_MAX_OUTPUT, POS_MAX_OUTPUT))
    }

    val isObstructing get() = pos >= ObstructionAngle

    val routine = SysIdRoutine(
        SysIdRoutine.Config(),
        SysIdRoutine.Mechanism(
            { volts: Voltage -> Elevator.setVoltage(volts.`in`(Volts))
            },
            { log: SysIdRoutineLog -> log.motor(armMotor.deviceId.toString())
                .voltage(Volts.of(armMotor.busVoltage))
                .angularPosition(Radians.of(pos))
                .angularVelocity(RadiansPerSecond.of(deltaAngle / 0.02))
            },
            this
        )
    )
    fun SysIDWrist(direction: Direction, quasistaic: Boolean) : Command {
        if(quasistaic) { return frc.robot.commands.elevator.routine.quasistatic(direction) }
        else { return frc.robot.commands.elevator.routine.dynamic(direction) }
    }
}