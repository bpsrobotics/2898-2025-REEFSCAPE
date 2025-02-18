package frc.robot.commands.swerve
import beaverlib.controls.TurningPID
import beaverlib.utils.Sugar.degreesToRadians
import beaverlib.utils.Sugar.radiansToDegrees
import beaverlib.utils.Sugar.within
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Drivetrain.swerveDrive
import frc.robot.subsystems.Vision
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

    private lateinit var usedTarget : PhotonTrackedTarget
    private var distToTag = 0.0
    private val runtime = Timer()
    val turningPID = TurningPID(0.1,0.01)
    val movementPID = PIDController(0.3, 0.0,0.05)
    val movementPID2 = PIDController(0.3, 0.0, 0.05)
    val offsetDist = sqrt(Vision.cameraOffset.x.pow(2) + Vision.cameraOffset.z.pow(2))
    var lastPose : Pose2d = Pose2d()
    val xOffset = 0.5
    val yOffset = horizontalOffset
    var trackedTagID = 0

    override fun initialize(){
        runtime.restart()
        lastPose = swerveDrive.pose
        val alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        trackedTagID = idAutoSelect(swerveDrive.pose, alliance)

    }

    override fun execute() {
//        val tagPose = aprilTagFieldInGame.getTagPose(usedTarget.fiducialId).get()
        val tagPose = aprilTagFieldLayout.getTagPose(2).get()
        //Update distance to tag with odometry update
        distToTag = sqrt(
            abs(-swerveDrive.pose.x - (tagPose.x + cos(tagPose.rotation.z)*xOffset + sin(tagPose.rotation.z)*yOffset)).pow(2) + abs(-swerveDrive.pose.y - (tagPose.y + sin(tagPose.rotation.z) *xOffset + cos(tagPose.rotation.z) * yOffset)).pow(2)
        )
        val currentRotation = swerveDrive.pose.rotation.degrees
        val desiredHeading = tagPose.rotation.z.radiansToDegrees() + 180
        val headingOffset = angleDist(realMod(desiredHeading, 360.0), realMod(currentRotation, 360.0))

        movementPID.setpoint = (tagPose.x + cos(tagPose.rotation.z)*xOffset + sin(tagPose.rotation.z)*yOffset)
        movementPID2.setpoint = (tagPose.y + sin(tagPose.rotation.z) *xOffset + cos(tagPose.rotation.z)*yOffset)
        val horizontalVelocity = -movementPID.calculate(-swerveDrive.pose.x)
        val verticalVelocity = -movementPID2.calculate(-swerveDrive.pose.y)
        val translation = Translation2d(horizontalVelocity, verticalVelocity)


//        turningPID.setPoint = desiredHeading // Set the desired value for the turningPID to the desired heading facing the tag
        // Set the desired value for the distance from the tag (Typically 0)
        // Desired rotational velocity, 0 when the rotation is within 3 degrees of the desired heading

        val angleVelocity = if (!currentRotation.within(8.0, desiredHeading)) { -headingOffset * 0.05 } else { 0.0 }
        speedConsumer(
            Transform2d(
                translation,
                Rotation2d(angleVelocity.degreesToRadians())
            )
        )
        SmartDashboard.putNumber("Dist to Tag", distToTag)
        SmartDashboard.putNumber("Horizontal Velocity", translation.y)
        SmartDashboard.putNumber("Vertical Velocity", translation.x)
        SmartDashboard.putNumber("Deired Heading", desiredHeading)
        SmartDashboard.putNumber("currentHeading", currentRotation)
        SmartDashboard.putNumber("angle Velocity", angleVelocity)

    }

    fun idAutoSelect(robotPose: Pose2d, alliance: Alliance): Int{
        var reefPose = Pose2d(Translation2d(0.0,0.0), Rotation2d())
        var tags = mutableListOf<Int>()
        if (alliance == Alliance.Red){
            reefPose = Pose2d(Translation2d(0.0,0.0), Rotation2d())
            tags = mutableListOf(1, 2)

        } else if(alliance == Alliance.Blue){
            reefPose = Pose2d(Translation2d(0.0,0.0), Rotation2d())
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
