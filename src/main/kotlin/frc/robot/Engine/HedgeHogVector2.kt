package beaverlib.utils.geometry
// File adapted from 2898's bpsrobotics engine
import edu.wpi.first.math.Num
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.asRotations
import beaverlib.utils.Units.Linear.*
import kotlin.math.*

class HedgeHogVector2(val x: Double, val y: Double) {
    companion object {
        @JvmName("FromNumber")
        fun new(x : Number, y : Number) = Vector2(x.toDouble(), y.toDouble())
        @JvmName("fromMagnitude&Angle")
        fun new(angle : AngleUnit, magnitude : Double) = Vector2(angle.cos()*magnitude, angle.sin()*magnitude)
        @JvmName("fromDistance")
        fun new(x: DistanceUnit, y: DistanceUnit) = Vector2(x.asMeters, y.asMeters)
        @JvmName("fromVelocity")
        fun new(x: VelocityUnit, y: VelocityUnit) = Vector2(x.asMetersPerSecond, y.asMetersPerSecond)
        @JvmName("fromAcceleration")
        fun new(x: Acceleration, y: Acceleration) = Vector2(x.asMetersPerSecondSquared, y.asFeetPerSecondSquared)


        fun zero() = Vector2(0.0, 0.0)
        fun crossProduct(vector1 : Vector2, vector2: Vector2 ) = (vector1.x * vector2.y) - (vector1.y * vector2.x)
        infix fun Vector2.dotProduct(other: Vector2) = (this.x * other.x) + (this.y * other.y)

    }
    constructor(pose: Pose2d) : this(pose.x, pose.y)
    constructor(angle: AngleUnit) : this(angle.cos(), angle.sin())
    /** @param angle Angle to rotate the vector by in radians*/
    fun rotateBy(angle: Double) : Vector2{
        return Vector2(cos(angle) *x - sin(angle) * y,
                        sin(angle)*x + cos(angle)*y)
    }
    /** @param angle Angle to rotate the vector by in radians*/
    fun rotateBy(angle: AngleUnit) : Vector2{
        return rotateBy(angle.asRadians)
    }


    /**
     * Distance from 0, 0, calculated using pythagorean theorem
     * */
    val magnitude get() = sqrt(x.pow(2) + y.pow(2))
    fun angle()         = AngleUnit(atan2(y,x))
    fun angleTo(other : Vector2)         = (this-other).angle()
    fun distance(other: Vector2): Double = (this-other).magnitude
    fun distance(pose: Pose2d): Double   = distance(Vector2(pose))
    fun xdistance(pos: Double): Double   = x - pos
    fun xdistance(pos: Vector2): Double  = x - pos.x
    fun xdistance(pos: Pose2d): Double   = x - pos.x
    fun ydistance(pos: Double): Double   = y - pos
    fun ydistance(pos: Vector2): Double  = y - pos.y
    fun ydistance(pos: Pose2d): Double   = y - pos.y

    operator fun plus(other: Vector2)  = Vector2(x + other.x, y + other.y)
    operator fun minus(other: Vector2) = Vector2(x - other.x,y - other.y)
    operator fun times(other: Double)  = Vector2(x * other,y * other)
    operator fun div(other: Double)    = Vector2(x / other,y / other)
    operator fun unaryMinus()          = Vector2(-x,-y)
    operator fun unaryPlus()           = this

    override fun toString() = "(x: ${x}, y: ${y})"

    /**
     * Reflects the point across a horizontal line
     * @return Reflected point
     * @param x The x value of the vertical line to reflect across
     * @author Ozy King
     */
    fun reflectHorizontally(x: Double) = Vector2(x + (x - this.x),y)

    /**
     * Creates a new Pose2d from the coordinate object and rotation
     * @param rotation The rotation of the pose, in degrees
     * @return A new Pose2d contructed from the coordinate and the rotation
     */
    fun toPose2d(rotation: Double) = Pose2d(x, y, Rotation2d.fromDegrees(rotation))
}