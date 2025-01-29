package frc.robot.commands.swerve
import beaverlib.controls.TurningPID
import beaverlib.utils.Sugar.radiansToDegrees
import beaverlib.utils.Sugar.within
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
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

class CoralAlignCommand(
    val speedConsumer: (Transform2d) -> Unit
) : Command() {
    private lateinit var usedTarget : PhotonTrackedTarget
    private var distX = 0.0
    private var distY = 0.0
    private var angleZ = 0.0
    private var yaw = 0.0
    private val runtime = Timer()
    val turningPID = TurningPID(0.1,0.01)
    val movementPID = PIDController(1.0, 0.0,0.1)


    init { addRequirements(Drivetrain) }

    override fun initialize(){
        runtime.reset()
        runtime.start()
        Vision.listeners.add("UpdateAlignCommand"){
            if (it.bestTarget != null) {
                usedTarget = it.bestTarget
                val trackedTarget = usedTarget.getBestCameraToTarget()
                distY = trackedTarget.y
                distX = trackedTarget.x
                yaw = usedTarget.yaw
                angleZ = trackedTarget.rotation.z.radiansToDegrees()
            }

        }

    }

    override fun execute() {
        if (!::usedTarget.isInitialized) return

        val offsetDist = sqrt(Vision.cameraOffset.x.pow(2) + Vision.cameraOffset.z.pow(2))
        val currentRotation = swerveDrive.pose.rotation.degrees
        val tagPose = aprilTagFieldInGame.getTagPose(usedTarget.fiducialId).get()
        val desiredHeading = tagPose.rotation.z.radiansToDegrees() + 180
        turningPID.setPoint = desiredHeading
        movementPID.setpoint = 0.0
        val angleVelocity = if (!desiredHeading.within(3.0, 0.0)) { turningPID.turnspeedOutput(currentRotation + 180) } else { 0.0 }
        val horizontalVelocity = if (!(distY + cos(currentRotation + 180 + Vision.cameraOffset.rotation.y) * offsetDist).within(0.1, 0.0)) { movementPID.calculate(distY) } else { 0.0 }
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