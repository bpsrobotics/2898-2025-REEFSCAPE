package frc.robot.subsystems

import beaverlib.utils.Sugar.clamp
import com.revrobotics.spark.*
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.ElevatorConstants.LOWER_LIMIT
import frc.robot.Constants.ElevatorConstants.MaxAccel
import frc.robot.Constants.ElevatorConstants.MaxVel
import frc.robot.Constants.ElevatorConstants.NEG_MAX_OUTPUT
import frc.robot.Constants.ElevatorConstants.POS_MAX_OUTPUT
import frc.robot.Constants.ElevatorConstants.UPPER_LIMIT
import frc.robot.Constants.ElevatorConstants.kD
import frc.robot.Constants.ElevatorConstants.kG
import frc.robot.Constants.ElevatorConstants.kI
import frc.robot.Constants.ElevatorConstants.kP
import frc.robot.Constants.ElevatorConstants.kS
import frc.robot.Constants.ElevatorConstants.kV
import frc.robot.RobotMap.ElevatorID1
import frc.robot.RobotMap.ElevatorID2
import frc.robot.RobotMap.ElevatorLeftMasterID
import frc.robot.RobotMap.ElevatorLeftSlaveID
import frc.robot.RobotMap.ElevatorRightMasterID
import frc.robot.RobotMap.ElevatorRightSlaveID
import frc.robot.RobotMap.LimitBotID
import frc.robot.RobotMap.LimitTopID

object Elevator : SubsystemBase() {
    private val leftMaster = SparkMax(ElevatorLeftMasterID, SparkLowLevel.MotorType.kBrushless)
    private val leftSlave = SparkMax(ElevatorLeftSlaveID, SparkLowLevel.MotorType.kBrushless)

    private val rightMaster = SparkMax(ElevatorRightMasterID, SparkLowLevel.MotorType.kBrushless)
    private val rightSlave = SparkMax(ElevatorRightSlaveID, SparkLowLevel.MotorType.kBrushless)

    private var elevatorConfig: SparkMaxConfig = SparkMaxConfig()

    private val elevEncoder = Encoder(ElevatorID1, ElevatorID2)
    private val botLimit = DigitalInput(LimitBotID)
    private val topLimit = DigitalInput(LimitTopID)


    private val constraints = TrapezoidProfile.Constraints(MaxVel, MaxAccel)
    var profile = TrapezoidProfile(constraints)
    var currentState = TrapezoidProfile.State(elevEncoder.distance, 0.0)
    var goalState = TrapezoidProfile.State(elevEncoder.distance, 0.0)

    val elevatorFeedforward = ElevatorFeedforward(kS, kG, kV)
    val elevatorPID = PIDController(kP,kI, kD)

    init {
        elevatorConfig
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(40)

        leftMaster.configure(
            elevatorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        leftSlave.configure(
            elevatorConfig.follow(leftMaster, true),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        rightMaster.configure(
            elevatorConfig.follow(leftMaster),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        rightSlave.configure(
            elevatorConfig.follow(rightMaster, true),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        defaultCommand = StabilizeElevator()
    }

    override fun periodic() {
        if (botLimit.get()) {
            //elevEncoder.reset() todo reset encoder
        } else if (topLimit.get()) {
        }

        SmartDashboard.putNumber("position elev", getPos())
    }

    fun getPos() : Double {
        return elevEncoder.distance
    }

    /** Run the motors toward [goalState].position at [targetSpeed] */
    private fun closedLoopMotorControl(targetSpeed : Double) {
        val outputPower = elevatorFeedforward.calculate(targetSpeed) + elevatorPID.calculate(getPos(), goalState.position)
        leftMaster.setVoltage(outputPower.clamp(NEG_MAX_OUTPUT, POS_MAX_OUTPUT))
    }
    /** Command to move the elevator to a goal position, command finishes once the trapezoid profile has finished*/
    class MoveElevator(val goalPosition : Double) : Command() {
        val timer = Timer()
        var targetSpeed : Double = 0.1
        init {addRequirements(Elevator)}
        override fun initialize() {
            if (goalPosition !in LOWER_LIMIT..UPPER_LIMIT) return
            timer.restart()
            currentState = TrapezoidProfile.State(getPos(), elevEncoder.rate)
            goalState = TrapezoidProfile.State(goalPosition, 0.0)
        }

        override fun execute() {
            targetSpeed = profile.calculate(timer.get(), currentState, goalState).velocity
            closedLoopMotorControl(targetSpeed)
        }

        override fun isFinished(): Boolean {
            return targetSpeed == 0.0
        }
    }
    /** Command to keep the elevator at the current goal position */
    class StabilizeElevator() : Command() {
        val timer = Timer()
        init {addRequirements(Elevator)}
        override fun initialize() {}

        override fun execute() { closedLoopMotorControl(0.0) }

        override fun isFinished(): Boolean { return false }
    }
    class DisableElevator() : Command() {
        init {addRequirements(Elevator)}
        override fun execute() { leftMaster.set(0.0) }
        override fun isFinished(): Boolean { return false }
    }
    class VoltageMove(val volts : Double, val time : Double) : Command() {
        val timer = Timer()
        init {addRequirements(Elevator)}

        override fun initialize() { timer.restart() }
        override fun execute() { leftMaster.set(volts) }
        override fun isFinished(): Boolean { return timer.hasElapsed(time) }
    }
    fun resetPos() {
        return elevEncoder.reset()
    }
}