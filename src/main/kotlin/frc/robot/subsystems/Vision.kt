package frc.robot.subsystems

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
import frc.robot.subsystems.Odometry.update
import frc.utils.odometry.hasTargets
import org.photonvision.targeting.PhotonPipelineResult
import java.io.File
import java.nio.file.FileSystem
import java.util.*

object Vision : SubsystemBase() {
    val cam = PhotonCamera("Camera_Module_v1")
    var results = mutableListOf<PhotonPipelineResult>()

    // Correct pose estimate with vision measurements

    override fun periodic(){
        results = cam.getAllUnreadResults()
        if (results.isEmpty()) return
        var result = results.get(results.size - 1);

    }


}