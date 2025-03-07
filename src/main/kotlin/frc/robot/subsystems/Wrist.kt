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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
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
    val encoder = DutyCycleEncoder(PivotPosID,2 * PI,0.88)

    var velocity = 0.0
    private val constraints = TrapezoidProfile.Constraints(Max_Velocity, Max_Accel)
    val encoderOffset = 0.0
    val profile = TrapezoidProfile(constraints)

    var deltaAngle = 0.0
    var lastPosition = pos

    var currentState = TrapezoidProfile.State(pos, 0.0)
    var goalState = TrapezoidProfile.State(pos, 0.0)

    val feedForward = ArmFeedforward(kS, kG, kV)
    val pid = PIDController(kP, kI, kD)
    val profiledPID = ProfiledPIDController(kP, kI, kD, constraints)

    val pos get() = angleModulus(encoder.get())

    var prev_target = pos

    init {
        pid.setpoint = pos

        // Wrist Motor Configuration
        wristConfig
            .smartCurrentLimit(30)
            .idleMode(SparkBaseConfig.IdleMode.kCoast)
        armMotor.configure(wristConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)

         SmartDashboard.putNumber("/wrist/pos", pos)
        SmartDashboard.putNumber("/wrist/vel", deltaAngle / 0.02)


        encoder.setInverted(true)
        defaultCommand = StabilizeWrist()

    }

    override fun periodic() {
        SmartDashboard.putNumber("/wrist/pos", pos)
        deltaAngle = pos - lastPosition
        lastPosition = pos
    }


    fun setVoltage(voltage: Double) {
        armMotor.setVoltage(voltage)
    }
    fun set(speed: Double) {
        armMotor.set(speed)
    }

    fun closedLoopPositionControl(targetSpeed: Double) {
        val outputPower = feedForward.calculate(pos, targetSpeed) + pid.calculate(pos, goalState.position)
        SmartDashboard.putNumber("/wrist/output_power", outputPower)
        armMotor.setVoltage(outputPower.clamp(NEG_MAX_OUTPUT, POS_MAX_OUTPUT))
    }

    fun profiledPIDControl(targetPos: Double) {
        profiledPID.setGoal(targetPos)
        val pidOutput = profiledPID.calculate(pos)
        val ffOutput = feedForward.calculate(pos, profiledPID.setpoint.velocity)
        val outputPower = pidOutput + ffOutput
        SmartDashboard.putNumber("/wrist/output_power", outputPower)
        SmartDashboard.putNumber("/wrist/targ_Speed", profiledPID.setpoint.velocity)
        armMotor.setVoltage(outputPower.clamp(NEG_MAX_OUTPUT, POS_MAX_OUTPUT))
    }

    fun isObstructing() : Boolean {
        return pos < ObstructionAngle
    }

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