package frc.robot.commands.swerve
import beaverlib.controls.TurningPID
import beaverlib.utils.Sugar.radiansToDegrees
import beaverlib.utils.Sugar.within
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Drivetrain.swerveDrive
import frc.robot.subsystems.Vision
import frc.robot.subsystems.aprilTagFieldInGame
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

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
    private var distToTag = Transform2d()
    private val runtime = Timer()
    val turningPID = TurningPID(0.1,0.01)
    val movementPID = PIDController(1.0, 0.0,0.1)
    val offsetDist = sqrt(Vision.cameraOffset.x.pow(2) + Vision.cameraOffset.z.pow(2))
    var lastPose : Pose2d = Pose2d()


    //init { addRequirements(Drivetrain) }

    override fun initialize(){
        runtime.restart()
        lastPose = swerveDrive.pose
        Vision.listeners.add("UpdateAlignCommand"){
            if (it.bestTarget != null) {
                usedTarget = it.bestTarget
                val trackedTarget = usedTarget.getBestCameraToTarget()
                distToTag = Transform2d(trackedTarget.x, trackedTarget.y, Rotation2d(trackedTarget.rotation.z))
            }

        }

    }
    /** Returns the distance from the tag to the center of the robot (Accounting for camera offset) */
    fun trueRobotToTag(): Double {
        return (distToTag.y + cos(swerveDrive.pose.rotation.degrees + 180 + Vision.cameraOffset.rotation.y) * offsetDist)
    }
    /** Given a field relative postion, return a double relating to the horizontal distance left and right of the given tag pose */

    fun fieldRelativeToTagRelative(tagPose : Pose3d, x : Double, y : Double) : Double {
        return Translation2d(x * cos(tagPose.rotation.y), y*sin(tagPose.rotation.y)).norm
    }
    /** Given a field relative postion, return a double relating to the horizontal distance left and right of the given tag pose */
    fun fieldRelativeToTagRelative(tagPose : Pose3d, translation : Transform2d) : Double {
        return Translation2d(translation.x * cos(tagPose.rotation.y), translation.y*sin(tagPose.rotation.y)).norm
    }
    /** Given a field relative postion, return a double relating to the horizontal distance left and right of the given tag pose */
    fun tagRelativeToFieldRelative(tagPose : Pose3d, x : Double) : Translation2d {
        return Translation2d(x * cos(tagPose.rotation.y), x * sin(tagPose.rotation.y))
    }
    override fun execute() {
        // If usedTarget is not initialized, the camera has not seen a tag yet, and should not begin alignment
        if (!::usedTarget.isInitialized) return
        val tagPose = aprilTagFieldInGame.getTagPose(usedTarget.fiducialId).get()
        //Update distance to tag with odometry update
        distToTag = Transform2d(distToTag.x, distToTag.y - fieldRelativeToTagRelative(tagPose, (lastPose - swerveDrive.pose)), distToTag.rotation)
        val currentRotation = (swerveDrive.pose.rotation.degrees + 180) % 360
        val desiredHeading = tagPose.rotation.z.radiansToDegrees() + 180

        turningPID.setPoint = desiredHeading // Set the desired value for the turningPID to the desired heading facing the tag
        movementPID.setpoint = horizontalOffset // Set the desired value for the distance from the tag (Typically 0)

        // Desired rotational velocity, 0 when the rotation is within 3 degrees of the desired heading
        val angleVelocity = if (!currentRotation.within(3.0, desiredHeading)) { turningPID.turnspeedOutput(currentRotation) } else { 0.0 }
        // Desired velocity vector, moves horizontally relative to the tag, but may be diagonal relative to the field
        val velocity = if (!trueRobotToTag().within(0.1, horizontalOffset)) { tagRelativeToFieldRelative(tagPose, movementPID.calculate(trueRobotToTag())) } else { Translation2d() }

        speedConsumer(
            Transform2d(
                velocity,
                Rotation2d(angleVelocity)
            )
        )
        SmartDashboard.putNumber("horizontalVelocity", velocity.x)
        SmartDashboard.putNumber("verticalVelocity", velocity.y)
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
