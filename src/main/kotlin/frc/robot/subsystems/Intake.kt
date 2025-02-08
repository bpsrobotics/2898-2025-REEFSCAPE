package frc.robot.subsystems

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DutyCycleEncoder

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import com.revrobotics.AbsoluteEncoder

import frc.robot.Constants.IntakeConstants.ks
import frc.robot.Constants.IntakeConstants.kv
import frc.robot.RobotMap.IntakeID
import frc.robot.RobotMap.IntakePosID

object Intake : SubsystemBase() {
    private val IntakeMotor = SparkMax(IntakeID, SparkLowLevel.MotorType.kBrushless)
    private val configIntake : SparkMaxConfig = SparkMaxConfig()

    var moving = false

    var velocity = 0.0
    var Intakeff = SimpleMotorFeedforward(ks, kv)

    init {
        configIntake
            .smartCurrentLimit(40)
    }
}