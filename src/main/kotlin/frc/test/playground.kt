package frc.test

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile


object playground {

}
fun main() {
    val constraints = TrapezoidProfile.Constraints(4.0, 2.0)
    val elevaff = ElevatorFeedforward(0.1, 0.42, 9.44)
    val pid = ProfiledPIDController(0.0, 0.0, 0.01, constraints)
    val armFF = ArmFeedforward(-0.078825, 0.86, 2.9)

    pid.reset(0.0)
    pid.setTolerance(0.01)
    pid.setGoal(1.0)
    println(pid.calculate(0.5))
    println(pid.setpoint.velocity)
    println(elevaff.calculate(pid.setpoint.velocity))


}