package frc.robot.subsystems

import com.revrobotics.spark.*
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.engine.utils.initMotorControllers
import frc.robot.Constants.ElevatorConstants.MaxAccel
import frc.robot.Constants.ElevatorConstants.MaxVel
import frc.robot.Constants.ElevatorConstants.kD
import frc.robot.Constants.ElevatorConstants.kG
import frc.robot.Constants.ElevatorConstants.kI
import frc.robot.Constants.ElevatorConstants.kP
import frc.robot.RobotMap.ElevatorID1
import frc.robot.RobotMap.ElevatorID2
import frc.robot.RobotMap.ElevatorLeftMasterID
import frc.robot.RobotMap.ElevatorLeftSlaveID
import frc.robot.RobotMap.ElevatorRightMasterID
import frc.robot.RobotMap.ElevatorRightSlaveID

object Elevator : SubsystemBase() {
    private val leftMaster = SparkMax(ElevatorLeftMasterID, SparkLowLevel.MotorType.kBrushless)
    private val leftSlave = SparkMax(ElevatorLeftSlaveID, SparkLowLevel.MotorType.kBrushless)

    private val rightMaster = SparkMax(ElevatorRightMasterID, SparkLowLevel.MotorType.kBrushless)
    private val rightSlave = SparkMax(ElevatorRightSlaveID, SparkLowLevel.MotorType.kBrushless)

    var elevatorConfig: SparkMaxConfig = SparkMaxConfig()

    val motors = arrayOf(leftSlave, leftMaster, rightMaster, rightSlave)

    private val elevEncoder = Encoder(ElevatorID1, ElevatorID2)

    private val constraints = TrapezoidProfile.Constraints(MaxVel, MaxAccel)
    var profile = TrapezoidProfile(constraints)
    var currentState = TrapezoidProfile.State(elevEncoder.distance, 0.0)
    var goalState = TrapezoidProfile.State(elevEncoder.distance, 0.0)

    val leftPIDController = leftMaster.closedLoopController

    var prevUpdateTime = 0.0

    var targetControl = false
    var elevPower = 0.0


    init {
        elevatorConfig
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAlternateOrExternalEncoder)
            .pid(kP, kI, kD)


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


    }
    fun getPos() : Double {
        return elevEncoder.distance
    }
    fun motorPeriodic() {
        val curTime = Timer.getFPGATimestamp()
        val dT = curTime - prevUpdateTime
        prevUpdateTime = curTime
        if (targetControl) {
            goalState.position = getPos()
            leftPIDController.setReference(
                elevEncoder.distance,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                kG,
                SparkClosedLoopController.ArbFFUnits.kVoltage
            )
        } else {
            currentState.position = getPos()
            currentState.velocity = 0.0
            leftMaster.set(elevPower)
        }
    }

    fun resetPos() {
        return elevEncoder.reset()
    }

}



