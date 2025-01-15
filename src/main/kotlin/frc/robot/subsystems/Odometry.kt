package frc.robot.subsystems

import beaverlib.odometry.vision
import com.team2898.engine.utils.odometry.Vision
import com.team2898.engine.utils.units.Degrees
import com.team2898.engine.utils.units.Meters
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine

object Odometry :  SubsystemBase(){
    private val vision = Vision("Camera_Module_v1")
    var defaultPose = Drivetrain.getPose()
    var Estimatedpose = Pose2d()

//    var poseA = Pose2d()
//    var poseB = Pose2d()

//    // WPILib
//    var publisher = NetworkTableInstance.getDefault()
//        .getStructTopic("MyPose", Pose2d.struct).publish()
//    var arrayPublisher = NetworkTableInstance.getDefault()
//        .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish()
//    fun supplyPose(): Pose2d {
//        return Pose2d(pose.x, pose.y, pose.rotation)
//    }

//    fun autoPose(): Pose2d {
//        return Pose2d(pose.x, pose.y, pose.rotation)
//    }


    /** Robot rotation speed in m/s */
    var velocity: Translation2d = Translation2d()
    private var lastPose = Pose2d()
    private val timer = Timer()

    fun zero(){
//        reset(Pose2d(0.0,0.0, Rotation2d.fromDegrees(0.0)))
        lastPose = defaultPose
    }

    fun setpoint(p: Pose2d) {
//        reset(p)
    }

    fun supplyEstimatedPose() {
        Drivetrain.addVisionMeasurement(Estimatedpose, timer.get())
    }

    init {
        timer.start()

        zero()
    }
    override fun reset(x: Meters, y: Meters, theta: Degrees) {
        val p = Pose2d(x.value, y.value, Rotation2d.fromDegrees(theta.value))
    }
    override fun periodic(){
        update()
    }
    override fun update(){



//        NavX.update(timer.get())
//        publisher.set(poseA)
//        arrayPublisher.set(arrayOf(poseA, poseB))
        val result  = vision.getEstimatedPose(lastPose)

        if (result == null){
            Estimatedpose = Drivetrain.getPose()
        } else {
            Estimatedpose = result.get().estimatedPose.toPose2d()
        }
        lastPose = Estimatedpose


//        poseA = pose
//        velocity = Translation2d((lastPose.x - pose.x)/timer.get(), (lastPose.y - pose.y)/timer.get())
//        lastPose = pose
//        SmartDashboard.putNumber("Odometry/FieldX", pose.x)
//        SmartDashboard.putNumber("Odometry/FieldY", pose.y)
//        SmartDashboard.putNumber("Odometry/Angle", pose.rotation.degrees)
//        SmartDashboard.putNumberArray("Odometry/velocity", arrayOf(velocity.x,velocity.y))
//        timer.reset()
    }


//    @Suppress("unused")
//    fun resetOdometry(newpose: Pose2d) {
//        pose = Pose2d(newpose.x, newpose.y, -newpose.rotation)
//
//    }
//
//    override fun initSendable(builder: SendableBuilder) {
//        SendableRegistry.setName(this, toString())
//        builder.addDoubleProperty("x", {pose.x}) {}
//        builder.addDoubleProperty("y", { pose.y }){}
//        builder.addDoubleProperty("rotation", { pose.rotation.radians }, null)
//    }
}