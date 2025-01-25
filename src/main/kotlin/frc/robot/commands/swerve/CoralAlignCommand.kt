package frc.robot.commands.swerve
import beaverlib.utils.Sugar.within
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Drivetrain.swerveDrive
import frc.robot.subsystems.Vision
import frc.robot.subsystems.aprilTagFieldLayout
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.cos
import kotlin.math.sin

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
            angleZ = trackedTarget.rotation.z

        }
    }

    override fun execute() {
        val currentRotation = swerveDrive.pose.rotation.degrees
        val angleVelocity = if (!yaw.within(10.0, 0.0)){-1.0 * yaw * 0.1 }else{0}.toDouble()
        val horizontalVelocity = if (!distY.within(0.1, 0.0)){-1.0 * distY * 0.1 }else{0}.toDouble()
        val tagPose = aprilTagFieldLayout.getTagPose(usedTarget.fiducialId).get()

        speedConsumer(Transform2d(
            horizontalVelocity * cos(tagPose.rotation.y),
            horizontalVelocity * sin(tagPose.rotation.y)
            , Rotation2d(angleVelocity)))
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        Vision.listeners.remove("UpdateAlignCommand")
    }




}