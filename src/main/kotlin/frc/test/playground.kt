package frc.test

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units
import kotlin.math.PI

object playground {

}
fun main() {
    val elevaff = ElevatorFeedforward(0.01, 0.01, 1.0)
    val armFF = ArmFeedforward(-0.078825, 0.86, 2.9)



    println(elevaff.calculate(0.00))


}