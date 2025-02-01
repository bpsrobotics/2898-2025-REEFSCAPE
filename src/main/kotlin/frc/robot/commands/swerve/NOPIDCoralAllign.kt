package frc.robot.commands.swerve
import beaverlib.utils.Sugar.degreesToRadians
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
import org.dyn4j.geometry.Rotation
import org.photonvision.targeting.PhotonTrackedTarget
import swervelib.SwerveController
import kotlin.math.*

class NOPIDCoralAlign(
    val speedConsumer: (Transform2d) -> Unit
) : Command() {
    private var usedTarget : PhotonTrackedTarget? = null
    private var distX = 0.0
    private var distY = 0.0
    private var angleZ = 0.0
    private var yaw = 0.0
    private val runtime = Timer()

    init {

    }

    override fun initialize(){
        runtime.reset()
        runtime.start()
        usedTarget = null
        Vision.listeners.add("UpdateAlignCommand"){
            if (it.bestTarget != null) {
                usedTarget = it.bestTarget
                val trackedTarget = it.bestTarget.getBestCameraToTarget()
                distY = trackedTarget.y
                distX = trackedTarget.x
                yaw = it.bestTarget.yaw
                angleZ = trackedTarget.rotation.z.radiansToDegrees()
            }

        }

    }

    override fun execute() {

        if (usedTarget != null) {
            val offsetDist = sqrt(Vision.cameraOffset.x.pow(2) + Vision.cameraOffset.z.pow(2))
            val currentRotation = (swerveDrive.pose.rotation.degrees + 180)
            val tagPose = aprilTagFieldInGame.getTagPose(usedTarget!!.fiducialId).get()
            val desiredHeading =  tagPose.rotation.y + 180.0
            val angleVelocity = if (!currentRotation.within(8.0, desiredHeading)) { (desiredHeading - currentRotation)  * 0.1 } else { 0.0 }
            val horizontalVelocity =
                if (!(distY).within(0.1, 0.0)) { -distY * 1.0 } else { 0.0 }


            speedConsumer(
                Transform2d(
                    horizontalVelocity * -sin(tagPose.rotation.y),
                    horizontalVelocity * cos(tagPose.rotation.y),
                    Rotation2d(angleVelocity)
                )
            )
            SmartDashboard.putNumber("horizontalVelocity", horizontalVelocity * cos(tagPose.rotation.y))
            SmartDashboard.putNumber("verticalVelocity", horizontalVelocity * sin(tagPose.rotation.y))
            SmartDashboard.putNumber("rotationalVelocity", angleVelocity)
            SmartDashboard.putNumber("currentrotation", currentRotation)
            SmartDashboard.putNumber("distY", distY)
            SmartDashboard.putBoolean("horizontal aligned", (distY + cos(currentRotation + Vision.cameraOffset.rotation.y) * offsetDist).within(0.1, 0.0))

            SmartDashboard.putNumber("tagID", usedTarget!!.fiducialId.toDouble())


        }
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        Vision.listeners.remove("UpdateAlignCommand")
        SmartDashboard.putNumber("horizontalVelocity", 0.0)
        SmartDashboard.putNumber("verticalVelocity", 0.0)
        speedConsumer(
            Transform2d(
                0.0,
                0.0,
                Rotation2d(0.0)
            )
        )
    }
}