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
import frc.robot.subsystems.Drivetrain.swerveDrive
import frc.robot.subsystems.Vision
import frc.robot.subsystems.aprilTagFieldInGame
import frc.robot.subsystems.aprilTagFieldLayout
import org.photonvision.PhotonUtils
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.*
import kotlin.math.*


/**
 * Align with the best tag, using odometry to supplement when the camera has not seen a tag.
 * @param speedConsumer Field relative drive function
 * @param horizontalOffset How far to the left or right you want to align with the tag
 */
class CoralAlignCommandWithOdometry(
    val speedConsumer: (Transform2d) -> Unit,
    val horizontalOffset : Double = 0.0
) : Command() {

    private lateinit var usedTarget : PhotonTrackedTarget
    private var distToTag = 0.0
    private val runtime = Timer()
    val turningPID = TurningPID(0.1,0.01)
    val movementPID = PIDController(1.0, 0.0,0.1)
    val offsetDist = sqrt(Vision.cameraOffset.x.pow(2) + Vision.cameraOffset.z.pow(2))
    var lastPose : Pose2d = Pose2d()
    val xOffset = 0.0
    val yOffset = 0.0
    var trackedTagID = 0
    val testTranslation = Translation2d(0.0, 0.0)



    //init { addRequirements(Drivetrain) }

    override fun initialize(){
        runtime.restart()
        lastPose = swerveDrive.pose
        val minzAngle = 180.0
        trackedTagID = idAutoSelect(swerveDrive.pose, DriverStation.Alliance.Red)

    }
    /** Returns the distance from the tag to the center of the robot (Accounting for camera offset) */
//    fun trueRobotToTag(): Double {
//        return (distToTag.y + cos(swerveDrive.pose.rotation.degrees + 180 + Vision.cameraOffset.rotation.y) * offsetDist)
//    }
    /** Given a field relative postion, return a double relating to the horizontal distance left and right of the given tag pose */

//    fun fieldRelativeToTagRelative(tagPose : Pose3d, x : Double, y : Double) : Double {
//        return Translation2d(x * cos(tagPose.rotation.y), y*sin(tagPose.rotation.y)).norm
//    }
    /** Given a field relative postion, return a double relating to the horizontal distance left and right of the given tag pose */
//    fun fieldRelativeToTagRelative(tagPose : Pose3d, translation : Transform2d) : Double {
//        return Translation2d(translation.x * cos(tagPose.rotation.y), translation.y*sin(tagPose.rotation.y)).norm
//    }
    /** Given a field relative postion, return a double relating to the horizontal distance left and right of the given tag pose */
//    fun tagRelativeToFieldRelative(tagPose : Pose3d, x : Double) : Translation2d {
//        return Translation2d(x * cos(tagPose.rotation.y), x * sin(tagPose.rotation.y))
//    }
    override fun execute() {
        // If usedTarget is not initialized, the camera has not seen a tag yet, and should not begin alignment
//        val tagPose = aprilTagFieldInGame.getTagPose(usedTarget.fiducialId).get()
        val tagPose = aprilTagFieldLayout.getTagPose(2).get()
        //Update distance to tag with odometry update
//        distToTag = Transform2d(distToTag.x, distToTag.y - fieldRelativeToTagRelative(tagPose, (lastPose - swerveDrive.pose)), distToTag.rotation)
        distToTag = sqrt(
            abs(-swerveDrive.pose.x - (tagPose.x + cos(tagPose.rotation.z)*xOffset + sin(tagPose.rotation.z)*yOffset)).pow(2) + abs(-swerveDrive.pose.y - (tagPose.y + sin(tagPose.rotation.z) *xOffset + cos(tagPose.rotation.z) * yOffset)).pow(2)
        )
        val currentRotation = (swerveDrive.pose.rotation.degrees)
        val desiredHeading = tagPose.rotation.z.radiansToDegrees() + 180
        val headingOffset = desiredHeading - currentRotation

        val horizontalVelocity = (-swerveDrive.pose.x - (tagPose.x + cos(tagPose.rotation.z)*xOffset + sin(tagPose.rotation.z)*yOffset))*distToTag
        val verticalVelocity = (-swerveDrive.pose.y - (tagPose.y + sin(tagPose.rotation.z) *xOffset + cos(tagPose.rotation.z)*yOffset))*distToTag
        val translation = Translation2d(horizontalVelocity, verticalVelocity)


//        turningPID.setPoint = desiredHeading // Set the desired value for the turningPID to the desired heading facing the tag
//        movementPID.setpoint = horizontalOffset // Set the desired value for the distance from the tag (Typically 0)
        // Desired rotational velocity, 0 when the rotation is within 3 degrees of the desired heading

        val angleVelocity = if (!currentRotation.within(8.0, desiredHeading)) { currentRotation/abs(currentRotation)*(desiredHeading - abs(currentRotation))  * 0.05 } else { 0.0 }
        // Desired velocity vector, moves horizontally relative to the tag, but may be diagonal relative to the field
//        val velocity = if (!trueRobotToTag().within(0.1, horizontalOffset)) { tagRelativeToFieldRelative(tagPose, movementPID.calculate(trueRobotToTag())) } else { Translation2d() }
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
            tags = mutableListOf(8, 9, 10, 11, 6, 7)

        } else if(alliance == Alliance.Blue){
            reefPose = Pose2d(Translation2d(0.0,0.0), Rotation2d())
            tags = mutableListOf(20, 19, 18, 17, 22, 21)
        }
        var currentAngle = atan2(swerveDrive.pose.x - reefPose.x, swerveDrive.pose.y - reefPose.y).radiansToDegrees()
        if (currentAngle<0){
            currentAngle += 360
        }
        when{
            currentAngle.within(30.0, 30.0) -> return tags[0]
            currentAngle.within(30.0, 90.0) -> return tags[1]
            currentAngle.within(30.0, 150.0) -> return tags[2]
            currentAngle.within(30.0, 210.0) -> return tags[3]
            currentAngle.within(30.0, 270.0) -> return tags[4]
            currentAngle.within(30.0, 330.0) -> return tags[5]
        }
        return 0
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
