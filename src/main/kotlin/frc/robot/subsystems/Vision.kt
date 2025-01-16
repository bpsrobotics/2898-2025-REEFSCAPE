package frc.robot.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.targeting.PhotonPipelineResult


val aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
val robotToCam = Transform3d(
    Translation3d(0.5, 0.0, 0.5),
    Rotation3d(0.0, 0.0, 0.0)
) //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

object Vision : SubsystemBase() {
    val cam = PhotonCamera("Camera_Module_v1")
    var results = mutableListOf<PhotonPipelineResult>()
    val listeners : MutableList<(PhotonPipelineResult) -> Unit> = mutableListOf<(PhotonPipelineResult) -> Unit>()
    var poseEstimator =
        PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam)
    // Correct pose estimate with vision measurements

    override fun periodic(){
        results = cam.getAllUnreadResults()
        if (results.isEmpty()) return
        results.forEach { visionResult: PhotonPipelineResult ->
            listeners.forEach { listener: (PhotonPipelineResult) -> Unit ->
                    listener(visionResult)
            }
        }
    }
    fun getRobotPosition(result: PhotonPipelineResult) : Pose3d? {
        val estimatedPose = poseEstimator.update(result) ?: return null
        if (estimatedPose.isEmpty) return null
        return estimatedPose.get().estimatedPose

    }
}