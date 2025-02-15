package frc.robot.subsystems

import beaverlib.utils.Sugar.degreesToRadians
import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.targeting.PhotonPipelineResult
import kotlin.math.abs


val aprilTagFieldLayout = AprilTagFieldLayout(
    mutableListOf(
        AprilTag(1, Pose3d(Translation3d(0.0,0.0,0.488), Rotation3d(0.0,0.0,0.0))),
        AprilTag(2, Pose3d(Translation3d(0.0, 1.0, 0.488), Rotation3d(0.0, 0.0, 0.0))),
        AprilTag(3, Pose3d(Translation3d(0.0, 2.0, 0.488), Rotation3d(0.0, 0.0, 0.0)))
    ),
    10.0,10.0)

val aprilTagFieldInGame = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)

    //AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
val robotToCam = Transform3d(
    Translation3d(-0.0762, 0.0, 0.5),
    Rotation3d(0.0, 0.0, 0.0)
) //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

val robotToCam2 = Transform3d(
    Translation3d(-0.0762, 0.0, 0.5),
    Rotation3d(0.0, 0.0, (180.0).degreesToRadians())
)

/**
 * A MutableMap used specifically for managing Lambdas
 * Specified Type is used as the input for argument for listener Lambdas
 */
data class Signal<Type>(
    val listeners : MutableMap<String, (Type) -> Unit> = mutableMapOf()
) {
    /**
     * Adds a listener with the given name. The function will be run whenever the parent object calls the update function
     * @param name String to denote the name of the listener, used to remove specific listeners later on
     * @param function The function to run when the listener updates */
    fun add(name: String, function: (Type) -> Unit){
        listeners.put(name, function)
    }

    /**
     * Removes a listener with the given name
     * @param name The name of the listener to remove */
    fun remove(name: String){
        listeners.remove(name)
    }

    /**
     * Runs all active listeners with the given input
     * @param input The input for each of the listening functions
     * */
    fun update(input : Type) {
        listeners.forEach {
            it.value(input)
        }
    }
}

object Vision : SubsystemBase() {
    val cam = PhotonCamera("Arducam_OV9281_USB_Camera")
    val cam2 = PhotonCamera("USB_Camera")
    val cameraOffset = robotToCam
    val cameraOffset2 = robotToCam2
    var results = mutableListOf<PhotonPipelineResult>()
    val listeners = Signal<PhotonPipelineResult>()
    var poseEstimator =
        PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam)
    var previousPose = Pose2d()

    init {
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE)
    }
    override fun periodic(){
        results = cam.allUnreadResults
        results.addAll(cam2.allUnreadResults)

        SmartDashboard.putBoolean("resultsIsEmpty", results.isEmpty())
        if (!results.isEmpty()) {
            // Iterate through each of the results
            results.forEach { visionResult: PhotonPipelineResult ->
                // Iterate through each of the listener functions, and call them passing the vision result as the input
                listeners.update(visionResult)

            }
        }
    }

    /**
     * Returns the estimated robot position given a PhotonPipelineResult
     * @param result The PhotonPipelineResult
     * @return The estimated Pose3d of the robot. If there is none, return null.
     */
    fun getRobotPosition(result: PhotonPipelineResult): Pose3d? {
        setReference(previousPose)

        val estimatedPose = poseEstimator.update(result) ?: return null

        if (estimatedPose.isEmpty) return null
        previousPose = estimatedPose.get().estimatedPose.toPose2d()
        return estimatedPose.get().estimatedPose

    }

    fun getValidID(result: PhotonPipelineResult): Int {
        val validIDs = mutableListOf(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22)
        var zAngle = 180.0
        var bestTarget = 0
        for (i in result.getTargets()){
            if (i.fiducialId in validIDs){
                val zAngleDiff = abs(180.0-i.getBestCameraToTarget().rotation.z)
                if ( zAngleDiff < zAngle){
                    bestTarget = i.fiducialId
                    zAngle = zAngleDiff
                }
            }
        }
        return bestTarget
    }

    /**
     * Returns the estimated robot position given a PhotonPipelineResult
     * @param pose The reference pose
     */
    fun setReference(pose: Pose2d): Unit {
        poseEstimator.setReferencePose(pose)
    }

    fun getStandardDev(): Matrix<N3,N1>{
        val stdv = Matrix(Nat.N3(), Nat.N1())
        stdv.set(0,0, 3.0)
        stdv.set(1,0, 3.0)
        stdv.set(2,0, 3.0)
        return stdv
    }
}