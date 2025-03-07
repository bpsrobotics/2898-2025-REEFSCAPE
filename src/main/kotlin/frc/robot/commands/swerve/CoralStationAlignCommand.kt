package frc.robot.commands.swerve
import beaverlib.controls.TurningPID
import beaverlib.utils.Sugar.degreesToRadians
import beaverlib.utils.Sugar.radiansToDegrees
import beaverlib.utils.Sugar.within
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Angular.radiansPerSecond
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Engine.angleDistanceTo
import frc.robot.Engine.angleDistanceWithin
import frc.robot.Engine.standardPosition
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Drivetrain.swerveDrive
import frc.robot.subsystems.Vision
import frc.robot.subsystems.aprilTagFieldInGame
import frc.robot.subsystems.aprilTagFieldLayout
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.*
import kotlin.jvm.optionals.getOrElse
import kotlin.math.*


/**
 * Align with the best tag, using odometry to supplement when the camera has not seen a tag.
 * @param speedConsumer Field relative drive function
 * @param horizontalOffset How far to the left or right you want to align with the tag
 */
class CoralStationAlignCommand(
    val speedConsumer: (Transform2d) -> Unit,
    val horizontalOffset : Double = 0.0
) : Command() {

    val movementPID = PIDController(1.5, 0.0,0.05)
    val movementPID2 = PIDController(1.5, 0.0, 0.05)
    val xOffset = 0.5
    var trackedTagID = 0
    var tagPose: Pose3d = Pose3d()
    var desiredHeading = swerveDrive.pose.rotation.radians.radians.standardPosition

    override fun initialize(){
        val alliance = DriverStation.getAlliance().orElse(Alliance.Red)
        trackedTagID = idAutoSelect(swerveDrive.pose, alliance)
        tagPose = aprilTagFieldInGame.getTagPose(trackedTagID).get()

        movementPID.setpoint = tagPose.x + cos(tagPose.rotation.z)*xOffset - sin(tagPose.rotation.z)*horizontalOffset
        movementPID2.setpoint = tagPose.y + sin(tagPose.rotation.z)*xOffset + cos(tagPose.rotation.z)*horizontalOffset
        desiredHeading = (tagPose.rotation.z+PI).radians.standardPosition

    }
    override fun execute() {
        SmartDashboard.putNumber("trackedTagID", trackedTagID.toDouble())

        val currentRotation = swerveDrive.pose.rotation.radians.radians
        val headingOffset = desiredHeading.angleDistanceTo(currentRotation.standardPosition)
        //calculate horizontalVelocity (speed moving side-ways to target)
        //caluclate verticalVelocity (speed moving towards target)

        val translation = Translation2d(movementPID.calculate(swerveDrive.pose.x), movementPID2.calculate(swerveDrive.pose.y))
//        turningPID.setPoint = desiredHeading // Set the desired value for the turningPID to the desired heading facing the tag
        // Set the desired value for the distance from the tag (Typically 0)
        // Desired rotational velocity, 0 when the rotation is within 3 degrees of the desired heading

        val angleVelocity = if (!currentRotation.angleDistanceWithin(8.0.degrees, desiredHeading))
        { (-headingOffset.asRadians *0.05).radiansPerSecond }
        else { 0.0.radiansPerSecond }

        speedConsumer(
            Transform2d(
                translation,
                Rotation2d(angleVelocity.asRadiansPerSecond)
            )
        )

    }

    fun idAutoSelect(robotPose: Pose2d, alliance: Alliance): Int{
        var tags = mutableListOf<Int>()
        if (alliance == Alliance.Red){
            tags = mutableListOf(1, 2)

        } else if(alliance == Alliance.Blue){
            tags = mutableListOf(12, 13)
        }
        when{
            robotPose.y > 3.5 -> return tags[0]
            robotPose.y < 3.5 -> return tags[1]
        }
        return tags[0]
    }

    fun angleDist(a: Double, b: Double): Double {
        val diff = b - a;
        if (diff <= 180.0) return diff;
        else return (diff % 180.0) - 180.0;
    }

    fun realMod(a: Double, b: Double): Double {
        val mod = a % b;
        if (mod >= 0) return mod;
        else return mod + b;
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        Vision.listeners.remove("UpdateAlignCommand")
        SmartDashboard.putNumber("horizontalVelocity", 0.0)
        SmartDashboard.putNumber("verticalVelocity", 0.0)
        speedConsumer( Transform2d(0.0, 0.0, Rotation2d(0.0) ) )
    }






}
