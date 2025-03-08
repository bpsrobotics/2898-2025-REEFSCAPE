package frc.robot.subsystems

import beaverlib.utils.Sugar.clamp
import beaverlib.utils.Units.Linear.inches
import com.revrobotics.spark.*
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.CounterBase
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
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
import frc.robot.commands.elevator.VoltageElevator
import frc.robot.commands.wrist.StopWrist
import kotlin.math.PI

object Elevator : SubsystemBase() {
    /** Main motor, all other motors follow this one */
    val leftMaster = SparkMax(ElevatorLeftMasterID, SparkLowLevel.MotorType.kBrushless)

    private val leftSlave = SparkMax(ElevatorLeftSlaveID, SparkLowLevel.MotorType.kBrushless)

    private val rightMaster = SparkMax(ElevatorRightMasterID, SparkLowLevel.MotorType.kBrushless)
    private val rightSlave = SparkMax(ElevatorRightSlaveID, SparkLowLevel.MotorType.kBrushless)

    private var elevatorConfig: SparkMaxConfig = SparkMaxConfig()

    /** Encoder on the elevator */
    val elevEncoder = Encoder(ElevatorID1, ElevatorID2, true, CounterBase.EncodingType.k1X)
    /** Limit switch at the bottom of the elevator */
    val botLimit = DigitalInput(LimitBotID)
    /** Limit switch at the top of the elevator*/
    val topLimit = DigitalInput(LimitTopID)

    /** Max velocity & acceleration for [TrapezoidProfile]*/
    private val constraints = TrapezoidProfile.Constraints(MaxVel, MaxAccel)
    /** Trapezoid Motion Profile, handles accelerating the elevator to max velocity, and decelerating it so that velocity = 0 when the elevator is at its target */
    var profile = TrapezoidProfile(constraints)
    /** The state of the elevator at [TrapezoidProfile] */
    var currentState = TrapezoidProfile.State(elevEncoder.distance, 0.0)
    /** The desired state (position and velocity) of the elevator */
    var goalState = TrapezoidProfile.State(elevEncoder.distance, 0.0)

    val elevatorFeedforward = ElevatorFeedforward(kS, kG, kV)
    val profiledPID = ProfiledPIDController(kP, kI,kD, constraints)
    val pid = PIDController(kP,kI, kD)

    init {
        // Init motor controls
        elevatorConfig
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(25)

        leftMaster.configure(
            elevatorConfig.inverted(true),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        leftSlave.configure(
            elevatorConfig.follow(leftMaster),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        rightMaster.configure(
            elevatorConfig.follow(leftMaster, true),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        rightSlave.configure(
            elevatorConfig.follow(leftMaster, true),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        // Configures encoder to return a distance of 1 meter for every 2048 pulses
        elevEncoder.distancePerPulse = ( 2 * PI * 0.945 *2).inches.asMeters / 2048

        profiledPID.reset(getPos())


        SmartDashboard.putBoolean("/Elevator/Top", topLimit.get())
        SmartDashboard.putBoolean("/Elevator/Bottom", botLimit.get())
        SmartDashboard.putNumber("/Elevator/Position", getPos())
        SmartDashboard.putNumber("/Elevator/Targ_Position", profiledPID.setpoint.position)
        SmartDashboard.putNumber("/Elevator/Rate", elevEncoder.rate)
        SmartDashboard.putNumber("/Elevator/Targ_Vel", profiledPID.setpoint.velocity)
        SmartDashboard.putNumber("/Elevator/Current", leftMaster.outputCurrent)
        defaultCommand = StabilizeElevator()
    }


    override fun periodic() {
        if (!botLimit.get()) {
            resetPos()
        }
        SmartDashboard.putBoolean("/Elevator/Top", topLimit.get())
        SmartDashboard.putBoolean("/Elevator/Bottom", botLimit.get())
        SmartDashboard.putNumber("/Elevator/Position", getPos())
        SmartDashboard.putNumber("/Elevator/Rate", elevEncoder.rate)
        SmartDashboard.putNumber("/Elevator/Current", leftMaster.outputCurrent)
        SmartDashboard.putNumber("/Elevator/Targ_Vel", profiledPID.setpoint.velocity)
        SmartDashboard.putNumber("/Elevator/Targ_Position", profiledPID.setpoint.position)


//        kG = SmartDashboard.getNumber("/Elevator/Voltage", kG)

    }
    /** Returns the elevator encoders distance*/
    fun getPos() : Double {
        return elevEncoder.distance
    }

    fun setVoltage(voltage: Double) {
        leftMaster.setVoltage(voltage)
    }
    /** Run the motors to hold the elevator at [getPos] position */
    fun closedLoopPositionControl() {
        val outputPower = elevatorFeedforward.calculate(0.0) + pid.calculate(getPos(), profiledPID.goal.position)
        if (!botLimit.get()) {outputPower.coerceAtLeast(0.0)} //If touching bottom limit switch, stop moving down
        if(!topLimit.get()) {outputPower.coerceAtMost(kG)} // If touching top limit switch, stop moving up
        leftMaster.setVoltage(outputPower.clamp(NEG_MAX_OUTPUT, POS_MAX_OUTPUT))
    }

    /** Run the Motors toward [targetPos] using a profiled pid controller **/
    fun profiledPIDControl(targetPos : Double) {
        SmartDashboard.putNumber("/Elevator/Targ_Position", targetPos)
        profiledPID.setGoal(targetPos)
        val pidOutput = profiledPID.calculate(getPos())
        val ffOutput = elevatorFeedforward.calculate(profiledPID.setpoint.velocity)
        SmartDashboard.putNumber("/Elevator/outputpower", pidOutput + ffOutput)
        SmartDashboard.putNumber("/Elevator/outputpid", pidOutput )
        SmartDashboard.putNumber("/Elevator/outputff", ffOutput)



        leftMaster.setVoltage(pidOutput + ffOutput)
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