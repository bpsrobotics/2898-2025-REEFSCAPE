package frc.robot.subsystems

import com.revrobotics.spark.SparkLowLevel
import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
 // bro this aint working but screw it lol
object TheFabledStateSpace : SubsystemBase() {
    val momentOfInertia = 0.0
    val gearingRatio = 1.0
    val m_motor = SparkMax(2, SparkLowLevel.MotorType.kBrushless)

    val flywheelPlant: LinearSystem<N1, N1, N1> = LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), momentOfInertia, gearingRatio)
    val m_observer : KalmanFilter<N1, N1, N1> = KalmanFilter(Nat.N1(), Nat.N1(), flywheelPlant, VecBuilder.fill(0.0), VecBuilder.fill(0.0), 0.02)
     val m_controller: LinearQuadraticRegulator<N1, N1, N1> = LinearQuadraticRegulator(flywheelPlant, VecBuilder.fill(0.0), VecBuilder.fill(0.0),0.02)
     val m_loop : LinearSystemLoop<N1, N1, N1> = LinearSystemLoop(flywheelPlant, m_controller, m_observer, 12.0, 0.02)
}