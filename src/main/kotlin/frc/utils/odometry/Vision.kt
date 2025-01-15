package frc.utils.odometry

import frc.robot.Constants
import frc.test.main
import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.*
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonTrackedTarget
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.photonvision.targeting.PhotonPipelineResult
import java.io.File
import java.nio.file.FileSystem
import java.util.*


val aprilTags1: MutableList<AprilTag> = mutableListOf(
    AprilTag(1,
        Pose3d(
            Translation3d(1.05,1.07,0.0),
            Rotation3d()
        )
    ),
    AprilTag(3,
        Pose3d(
            Translation3d(0.15,1.07,0.0),
            Rotation3d()
        )
    )
)
val testLayout1 = AprilTagFieldLayout(aprilTags1,10.0,5.0)

class Vision (
    CameraName: String
) {
//    val cam = PhotonCamera(CameraName)

    var robotToCam = Transform3d(
        Translation3d(0.5, 0.0, 0.5),
        Rotation3d(0.0, 0.0, 0.0)
    )

    val aprilTagFieldLayout = AprilTagFieldLayout(Constants.VisionConstants.APRILTAG_FIELD.toPath())


    val PoseEstimator = PhotonPoseEstimator(
        aprilTagFieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        cam,
        robotToCam
    )
    fun getCameraData(targetID:Int) :  Transform3d{
        var result = cam.getLatestResult();
        val targets = result.getTargets() ?: return Transform3d(0.0, 0.0, 0.0, Rotation3d())
        for (i in targets) {
            if (i.fiducialId == targetID) {
                return i.bestCameraToTarget
            }
        }
        return result.bestTarget.bestCameraToTarget ?: return Transform3d(0.0,0.0,0.0, Rotation3d())
    }

    fun getCameraYaw(targetID: Int) : Double{

        val targets = result.getTargets() ?: return 0.0
        for (i in targets) {
            if (i.fiducialId == targetID) {
                return i.yaw
            }
        }
        return result.bestTarget.yaw ?: return 0.0
    }
    fun hasSpecificTarget(tagID:Int) : Boolean {
        val result = cam.latestResult
        if (result.hasTargets()) {
            var result = cam.getAllUnreadResults()
            result.filter { it.timestampSeconds } }
            for (i in targets) {
                if (i.fiducialId == tagID) {
                    return true
                }
            }
        }
        return false
    }

    fun hasTargets() : Boolean{
        val result = cam.latestResult
        return result.hasTargets()

    }

    fun getAllTrackedTargets() : List<PhotonTrackedTarget>{
        val result = cam.latestResult
        var targets: List<PhotonTrackedTarget> = result.getTargets()
        return targets
    }

    fun getSpecificTag(tagId: Int) : MutableList<PhotonTrackedTarget> {
        val result = cam.latestResult
        return result.getTargets()
    }


    fun getEstimatedPose(prevEstimatedRobotPose: Pose2d?): Optional<EstimatedRobotPose>? {
        if(prevEstimatedRobotPose != null) PoseEstimator.setReferencePose(prevEstimatedRobotPose)
        val pose = PoseEstimator.update() ?: return null
        return pose
    }
    fun getTagPoseFromField(targetID: Int) : Optional<Pose3d>?{

        val pose = aprilTagFieldLayout.getTagPose(targetID) ?: return null
        return pose
    }
}