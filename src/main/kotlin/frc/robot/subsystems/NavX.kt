package frc.robot.subsystems
import beaverlib.controls.TurningPID
import beaverlib.utils.Sugar.degreesToRadians
import beaverlib.utils.Units.Angular.asDegrees
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Angular.radiansPerSecond
import beaverlib.utils.Units.seconds
import beaverlib.utils.Units.toSeconds
import com.studica.frc.AHRS
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase

/** Container object for an instantiated NavX class, as well as other functions relating to the gyroscope */
object NavX : SubsystemBase() {
    /** Gyroscope used on robot */
    var navx = AHRS(AHRS.NavXComType.kMXP_SPI)
    var totalRotation = 0.0.radians
    private var lastRotation = 0.0.radians
    var rotationalSpeed = 0.0.radiansPerSecond

    init {
        navx.angleAdjustment = 0.0
    }

    /** @return The NavX's angle multiplied by -1 */
    fun getInvertedAngle(): Double{
        return -navx.angle
    }
    fun getAngle(): Double {
        return navx.angle
//        .plus(90.0).mod(360.0)
    }
    fun update(timeSinceUpdate: Double){
        totalRotation += TurningPID.minCircleDist(navx.angle.degrees.asRadians, lastRotation.asRadians).radians
        rotationalSpeed = TurningPID.minCircleDist(navx.angle.degrees.asRadians, lastRotation.asRadians).radians/timeSinceUpdate.toSeconds
        lastRotation = navx.angle.radians
        SmartDashboard.putNumber("Odometry/TotalRotation", totalRotation.asDegrees)
        SmartDashboard.putNumber("Odometry/Rotation", -navx.angle)

    }
    fun reset(){
        navx.reset()
        navx.angleAdjustment = -90.0
        totalRotation = 0.0.radians
    }
}