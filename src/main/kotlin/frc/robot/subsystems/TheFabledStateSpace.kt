package frc.robot.subsystems

import com.revrobotics.spark.SparkLowLevel
import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.*
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.Timer
import kotlin.math.abs

// bro this aint working but screw it lol, just a demo and not meant to be taken seriously
object TheFabledStateSpace : SubsystemBase() {
    var timer = 0.0
    val dt = Timer.getFPGATimestamp()
    var input = 0.0
    val momentOfInertia = 0.032 //kg/m^2
    val gearingRatio = 1.0
     val m_encoder = DutyCycleEncoder(0)
    val m_motor = SparkMax(2, SparkLowLevel.MotorType.kBrushless)
    var desiredVel = 0.0
    var desiredAccel = 0.0

    val flywheelPlant: LinearSystem<N1, N1, N1> = LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), momentOfInertia, gearingRatio)
    val m_observer : KalmanFilter<N1, N1, N1> = KalmanFilter(Nat.N1(), Nat.N1(), flywheelPlant, VecBuilder.fill(0.0), VecBuilder.fill(0.0), 0.02)
     val m_controller: LinearQuadraticRegulator<N1, N1, N1> = LinearQuadraticRegulator(flywheelPlant, VecBuilder.fill(0.0), VecBuilder.fill(0.0),0.02)
     val m_loop : LinearSystemLoop<N1, N1, N1> = LinearSystemLoop(flywheelPlant, m_controller, m_observer, 12.0, 0.02)
    fun timeDiff() :Double {
        val diff = abs(timer - dt)
        timer = dt
        return diff
    }
    fun getRate() : Double {
        val rate = m_encoder.get() / timeDiff()
        return rate
    }
    fun getAcceleration() : Double {
        val accel = getRate() / timeDiff()
        return accel
    } //this fun kinda redundant but im keeping it for now
    fun twoStateFF(): Matrix<N1, N1> {
        // ok so basically we taking u = B.pseudoinverse(accel. - AMatrix*velocityOrRate)
        val VelVec = VecBuilder.fill(getRate())
        val RkPlusOne = VelVec.times(getAcceleration().times(timeDiff()))
        val noInvKff = flywheelPlant.b.transpose().times(VecBuilder.fill(0.0)).times(m_controller.r)
        val realKff = noInvKff.inv().times(flywheelPlant.b.transpose().times(VecBuilder.fill(0.0)))
        val unfinishedUff = RkPlusOne.minus(flywheelPlant.a.times(getRate()))
        val Uff = realKff.times(unfinishedUff)
        return Uff
    }

    override fun periodic() {
        m_loop.setNextR(input)

        m_loop.correct(VecBuilder.fill(getRate()))
        m_loop.predict(0.020)

//        val voltage = m_loop.getU(0)
//        m_motor.setVoltage(voltage)
    }
    fun setNewVoltage() {
        val voltage = m_loop.getU(0) + twoStateFF().get(0,0)
        m_motor.setVoltage(voltage)
    }
}