package frc.test

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile


object playground {

}
fun main() {
    val constraints = TrapezoidProfile.Constraints(2.0, 4.0)
    val elevaff = ElevatorFeedforward(0.01, 0.01, 1.0)
    val pid = PIDController(1.0, 0.0, 0.01)
    val armFF = ArmFeedforward(-0.078825, 0.86, 2.9)

    pid.reset()
//    pid.setGoal(1.0)
    println(pid.calculate(0.0, 0.01))
//    println(pid.setpoint.velocity)
//    println(elevaff.calculate(pid.setpoint.velocity))


}