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
class ReefAlignCommand(
    val speedConsumer: (Transform2d) -> Unit,
    val horizontalOffset : Double = 0.0
) : Command() {

    private var distToTag = 0.0
    private val runtime = Timer()
    val turningPID = TurningPID(0.1,0.01)
    val movementPID = PIDController(1.5, 0.0,0.05)
    val movementPID2 = PIDController(1.5, 0.0, 0.05)
    var lastPose : Pose2d = Pose2d()
    val xOffset = 0.5
    val yOffset = horizontalOffset
    var trackedTagID = 0

    override fun initialize(){
        runtime.restart()
        lastPose = swerveDrive.pose
        val alliance = DriverStation.getAlliance().orElse(Alliance.Red)
        trackedTagID = idAutoSelect(swerveDrive.pose, alliance)

    }
    var tRoll = 0.0
    var tPitch = 0.0
    var tYaw = aprilTagFieldInGame.getTagPose(11).get().rotation.z
    var x = 0.5
    var y = 1.0
    override fun execute() {
        val tagPose = aprilTagFieldInGame.getTagPose(trackedTagID).get()
        SmartDashboard.putNumber("trackedTagID", trackedTagID.toDouble())
//        val tagPose = aprilTagFieldLayout.getTagPose(2).get()
//        SmartDashboard.putNumber("targetPose/Roll", tRoll)
//        SmartDashboard.putNumber("targetPose/Pitch", tPitch)
//        SmartDashboard.putNumber("Yaw", tYaw)
//        SmartDashboard.putNumber("x", x)
//        SmartDashboard.putNumber("y", y)
//        val tagPose = Pose3d(Translation3d(x, y, 0.488), Rotation3d(tRoll, tPitch, tYaw))

        //Update distance to target with odometry update
        distToTag = sqrt(
            (-swerveDrive.pose.x - (tagPose.x + cos(tagPose.rotation.z)*xOffset + cos(tagPose.rotation.z+PI/2)*yOffset)).pow(2) + (-swerveDrive.pose.y - (tagPose.y + sin(tagPose.rotation.z)*xOffset + sin(tagPose.rotation.z+PI/2)*yOffset)).pow(2)
        )

        val currentRotation = swerveDrive.pose.rotation.degrees
        val desiredHeading = (tagPose.rotation.z.radiansToDegrees() + 180) % 360
        val headingOffset = angleDist(desiredHeading, realMod(currentRotation, 360.0))

        //calculate horizontalVelocity (speed moving side-ways to target)
        movementPID.setpoint = tagPose.x + cos(tagPose.rotation.z)*xOffset + cos(tagPose.rotation.z+PI/2)*yOffset
        val horizontalVelocity = -movementPID.calculate(-swerveDrive.pose.x)

        //caluclate verticalVelocity (speed moving towards target)
        movementPID2.setpoint = tagPose.y + sin(tagPose.rotation.z)*xOffset + sin(tagPose.rotation.z+PI/2)*yOffset
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
            SmartDashboard.putNumber("Tag Heading", tagPose.rotation.z)
            SmartDashboard.putNumber("Deired Heading", desiredHeading)
            SmartDashboard.putNumber("currentHeading", realMod(currentRotation, 360.0))
            SmartDashboard.putNumber("angle Velocity", angleVelocity)

    }
    //automatically finds the face of the reef that robot is closest to and returns the ID of the apriltag on that face of the reef
    fun idAutoSelect(robotPose: Pose2d, alliance: Alliance): Int{
        var reefPose = Pose2d(Translation2d(0.0,0.0), Rotation2d())
        var tags = mutableListOf<Int>()
        if (alliance == Alliance.Red){
            reefPose = Pose2d(Translation2d(13.06185,4.03), Rotation2d())
            tags = mutableListOf(8, 9, 10, 11, 6, 7)

        } else if(alliance == Alliance.Blue){
            reefPose = Pose2d(Translation2d(0.0,0.0), Rotation2d())
            tags = mutableListOf(20, 19, 18, 17, 22, 21)
        }
        var currentAngle = atan2(-swerveDrive.pose.x - reefPose.x, -swerveDrive.pose.y - reefPose.y).radiansToDegrees()
        if (currentAngle<0){
            currentAngle += 360
        }
        SmartDashboard.putNumber("currentAngle", currentAngle)
        when{
            currentAngle.within(30.0, 30.0) -> return tags[0]
            currentAngle.within(30.0, 90.0) -> return tags[5]
            currentAngle.within(30.0, 150.0) -> return tags[4]
            currentAngle.within(30.0, 210.0) -> return tags[3]
            currentAngle.within(30.0, 270.0) -> return tags[2]
            currentAngle.within(15.0, 330.0) -> return tags[1]
        }
        return tags[0]
    }
    //calculate distance between two angles, works with 0-360 angle modulus
    fun angleDist(a: Double, b: Double): Double {
        val diff = b - a;
        if (diff <= 180.0) return diff;
        else return (diff % 180.0) - 180.0;
    }
    //converts other angle modulus formats to 0-360 format
    fun realMod(a: Double, b: Double): Double {
        val mod = a % b;
        if (mod >= 0) return mod;
        else return mod + b;
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        SmartDashboard.putNumber("horizontalVelocity", 0.0)
        SmartDashboard.putNumber("verticalVelocity", 0.0)
        speedConsumer( Transform2d(0.0, 0.0, Rotation2d(0.0) ) )
    }






}
