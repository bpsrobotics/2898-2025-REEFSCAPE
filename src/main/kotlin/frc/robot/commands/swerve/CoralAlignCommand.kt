package frc.robot.commands.swerve
import beaverlib.utils.Sugar.radiansToDegrees
import beaverlib.utils.Sugar.within
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Drivetrain.swerveDrive
import frc.robot.subsystems.Vision
import frc.robot.subsystems.aprilTagFieldInGame
import frc.robot.subsystems.aprilTagFieldLayout
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

class CoralAlignCommand(
    val speedConsumer: (Transform2d) -> Unit
) : Command() {
    private lateinit var usedTarget : PhotonTrackedTarget
    private var distX = 0.0
    private var distY = 0.0
    private var angleZ = 0.0
    private var yaw = 0.0
    private val runtime = Timer()
    private val swerve: Drivetrain


    init {
        this.swerve = Drivetrain
    }

    override fun initialize(){
        runtime.reset()
        runtime.start()
        Vision.listeners.add("UpdateAlignCommand"){
            usedTarget = it.bestTarget
            val trackedTarget = usedTarget.getBestCameraToTarget()
            distY = trackedTarget.y
            distX = trackedTarget.x
            yaw = usedTarget.yaw
            angleZ = trackedTarget.rotation.y.radiansToDegrees()
        }

    }

    override fun execute() {

        if (::usedTarget.isInitialized) {
            val offsetDist = sqrt(Vision.cameraOffset.x.pow(2) + Vision.cameraOffset.z.pow(2))
            val currentRotation = swerveDrive.pose.rotation.degrees
            val tagPose = aprilTagFieldInGame.getTagPose(usedTarget.fiducialId).get()
            val desiredHeading = tagPose.rotation.y.radiansToDegrees() + 180
//            val angleVelocity = if (!desiredHeading.within(10.0, 0.0)) { -1.0 * (desiredHeading - (currentRotation + 180)) * 0.1 } else { 0.0 }\
            val angleVelocity = 0.0
            val horizontalVelocity =
                if (!(distY + cos(currentRotation + 180 + Vision.cameraOffset.rotation.y) * offsetDist).within(0.1, 0.0)) { -1.0 * distY * 0.5 } else { 0.0 }
            speedConsumer(
                Transform2d(
                    horizontalVelocity * cos(tagPose.rotation.y),
                    horizontalVelocity * sin(tagPose.rotation.y),
                    Rotation2d(angleVelocity)
                )
            )
            SmartDashboard.putNumber("horizontalVelocity", horizontalVelocity * cos(tagPose.rotation.y))
            SmartDashboard.putNumber("verticalVelocity", horizontalVelocity * sin(tagPose.rotation.y))
        }
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        Vision.listeners.remove("UpdateAlignCommand")
    }




}