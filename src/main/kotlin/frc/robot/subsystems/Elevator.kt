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
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.ElevatorConstants.MaxAccel
import frc.robot.Constants.ElevatorConstants.MaxVel
import frc.robot.Constants.ElevatorConstants.NEG_MAX_OUTPUT
import frc.robot.Constants.ElevatorConstants.POS_MAX_OUTPUT
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

    var elevatorConfig: SparkMaxConfig = SparkMaxConfig()

    private val elevEncoder = Encoder(ElevatorID1, ElevatorID2)
    private val botLimit = DigitalInput(LimitBotID)
    private val topLimit = DigitalInput(LimitTopID)


    private val constraints = TrapezoidProfile.Constraints(MaxVel, MaxAccel)
    var profile = TrapezoidProfile(constraints)
    var currentState = TrapezoidProfile.State(elevEncoder.distance, 0.0)
    var goalState = TrapezoidProfile.State(elevEncoder.distance, 0.0)

    val elevFF = ElevatorFeedforward(kS, kG, kV)
    val elevPID = PIDController(kP,kI, kD)

    var prevUpdateTime = 0.0

    var targetControl = false
    var targSpeed = 0.0
    var outputPower = 0.0
    var rawOutput = 0.0


    init {
        elevatorConfig
            .idleMode(SparkBaseConfig.IdleMode.kBrake)

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

    override fun periodic() {
        if (botLimit.get()) {
            resetPos()
        }
        motorPeriodic()
    }

    fun getPos() : Double {
        return elevEncoder.distance
    }
    fun motorPeriodic() {
        val curTime = Timer.getFPGATimestamp()
        val dT = curTime - prevUpdateTime
        prevUpdateTime = curTime
        if (targetControl) {
            targSpeed = profile.calculate(dT, currentState, goalState).velocity
            outputPower = elevFF .calculate(targSpeed)
            outputPower += elevPID.calculate(getPos(), goalState.position)
            leftMaster.setVoltage(outputPower.clamp(NEG_MAX_OUTPUT, POS_MAX_OUTPUT))
        } else {
            currentState.position = getPos()
            currentState.velocity = 0.0
            leftMaster.set(rawOutput)
        }
    }

    fun resetPos() {
        return elevEncoder.reset()
    }


}



