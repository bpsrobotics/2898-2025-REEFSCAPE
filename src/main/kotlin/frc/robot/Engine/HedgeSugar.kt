package frc.robot.Engine

import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.radians
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.sign
/** Wraps the angle such that it is between 0 and 360 degrees, 0 degrees is the left side of the circle, and angle increases counterclockwise*/
inline val AngleUnit.standardPosition: AngleUnit
    get() =
        if (this.asRadians >= 0.0) { AngleUnit(this.asRadians % (2 * PI)) }
        else { AngleUnit((2 * PI) + (this.asRadians % (2 * PI))) }
fun AngleUnit.angleDistanceTo(other: AngleUnit): AngleUnit {
    val normal = this - other
    val wrap = -((2 * PI).radians * normal.asRadians.sign - normal)
//            println("normal: ${normal} wrap: ${wrap} sign: ${normal.sign} angleA: $angleA angleB: $angleB")
    return if (normal.asRadians.absoluteValue < wrap.asRadians.absoluteValue) { normal }
    else { wrap }
}
fun AngleUnit.angleDistanceWithin(maxError : AngleUnit, target : AngleUnit): Boolean {
    return this.angleDistanceTo(target).asRadians.absoluteValue < maxError.asRadians
}
class HedgeSugar {

}