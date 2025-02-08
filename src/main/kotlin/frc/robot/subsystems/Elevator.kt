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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
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
import frc.robot.commands.elevator.StabilizeElevator
object Elevator : SubsystemBase() {
    /** Main motor, all other motors follow this one */
    val leftMaster = SparkMax(ElevatorLeftMasterID, SparkLowLevel.MotorType.kBrushless)

    private val leftSlave = SparkMax(ElevatorLeftSlaveID, SparkLowLevel.MotorType.kBrushless)

    private val rightMaster = SparkMax(ElevatorRightMasterID, SparkLowLevel.MotorType.kBrushless)
    private val rightSlave = SparkMax(ElevatorRightSlaveID, SparkLowLevel.MotorType.kBrushless)

    private var elevatorConfig: SparkMaxConfig = SparkMaxConfig()

    /** Encoder on the elevator */
    val elevEncoder = Encoder(ElevatorID1, ElevatorID2)
    /** Limit switch at the bottom of the elevator */
    private val botLimit = DigitalInput(LimitBotID)
    /** Limit switch at the top of the elevator*/
    private val topLimit = DigitalInput(LimitTopID)

    /** Max velocity & acceleration for [TrapezoidProfile]*/
    private val constraints = TrapezoidProfile.Constraints(MaxVel, MaxAccel)
    /** Trapezoid Motion Profile, handles accelerating the elevator to max velocity, and decelerating it so that velocity = 0 when the elevator is at its target */
    var profile = TrapezoidProfile(constraints)
    /** The state of the elevator at [TrapezoidProfile] */
    var currentState = TrapezoidProfile.State(elevEncoder.distance, 0.0)
    /** The desired state (position and velocity) of the elevator */
    var goalState = TrapezoidProfile.State(elevEncoder.distance, 0.0)

    val elevatorFeedforward = ElevatorFeedforward(kS, kG, kV)
    val pid = PIDController(kP,kI, kD)

    init {
        // Init motor controls
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
        elevEncoder.distancePerPulse = 1/1.889
        defaultCommand = StabilizeElevator()
    }

    override fun periodic() {
        if (botLimit.get()) {
            elevEncoder.reset()
        }

        SmartDashboard.putNumber("/Elevator/Position", getPos())
        SmartDashboard.putNumber("/Elevator/Rate", elevEncoder.rate)

    }
    /** Returns the elevator encoders distance*/
    fun getPos() : Double {
        return elevEncoder.distance
    }

    /** Run the motors toward [goalState].position at [targetSpeed] */
    fun closedLoopMotorControl(targetSpeed : Double) {
        val outputPower = elevatorFeedforward.calculate(targetSpeed) + pid.calculate(getPos())
        if (botLimit.get()) {outputPower.coerceAtLeast(0.0)} //If touching bottom limit switch, stop moving down
        if(topLimit.get()) {outputPower.coerceAtMost(kG)} // If touching top limit switch, stop moving up
        leftMaster.setVoltage(outputPower.clamp(NEG_MAX_OUTPUT, POS_MAX_OUTPUT))
    }
    /** Resets the elevator encoder */
    fun resetPos() {
        return elevEncoder.reset()
    }

    /** Returns height percent from 0.0 to 1.0 **/
    fun heightPercent() : Double {
        return (getPos() / UPPER_LIMIT).clamp(0.0, 1.0)
    }

}