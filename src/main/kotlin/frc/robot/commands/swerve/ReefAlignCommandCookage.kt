package frc.robot.commands.swerve
import beaverlib.controls.TurningPID
import beaverlib.utils.Sugar.TAU
import beaverlib.utils.Sugar.degreesToRadians
import beaverlib.utils.Sugar.radiansToDegrees
import beaverlib.utils.Sugar.within
import beaverlib.utils.Units.Angular.*
import beaverlib.utils.Units.Linear.asInches
import beaverlib.utils.Units.Linear.meters
import beaverlib.utils.geometry.HedgeHogVector2
import beaverlib.utils.geometry.Vector2
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Engine.angleDistanceTo
import frc.robot.Engine.angleDistanceWithin
import frc.robot.Engine.standardPosition
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Drivetrain.swerveDrive
import frc.robot.subsystems.Vision
import frc.robot.subsystems.aprilTagFieldInGame
import frc.robot.subsystems.aprilTagFieldLayout
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.*
import kotlin.jvm.optionals.getOrElse
import kotlin.math.*


/**
 * Align with the best tag, using odometry to supplement when the camera has not seen a tag.
 * @param speedConsumer Field relative drive function
 * @param horizontalOffset How far to the left or right you want to align with the tag
 */
class ReefAlignCommandCookage(
    val speedConsumer: (Transform2d) -> Unit,
    val horizontalOffset : Double = 0.0
) : Command() {

    private val runtime = Timer()
    val turningPID = TurningPID(0.1,0.01)
    val horizontalMovementPID = PIDController(1.5, 0.0,0.05)
    val verticalMovementPID = PIDController(1.5, 0.0, 0.05)

    var lastPose : Pose2d = Pose2d()
    val xOffset = 0.5
    val yOffset = horizontalOffset
    var trackedTagID = 0
    var tagPose : Pose3d = Pose3d()


    override fun initialize(){
        runtime.restart()
        lastPose = swerveDrive.pose
        val alliance = DriverStation.getAlliance().orElse(Alliance.Red)
        trackedTagID = idAutoSelect(swerveDrive.pose, alliance)
        tagPose = aprilTagFieldInGame.getTagPose(trackedTagID).get()

        val offset = HedgeHogVector2(xOffset, yOffset)
        val goalPosition = HedgeHogVector2(tagPose.toPose2d()) + offset.rotateBy(tagPose.rotation.z)

        horizontalMovementPID.setpoint = goalPosition.x
        verticalMovementPID.setpoint = goalPosition.y

    }
    override fun execute() {

        SmartDashboard.putNumber("trackedTagID", trackedTagID.toDouble())

        //Update distance to target with odometry update

        val currentRotation = swerveDrive.pose.rotation.radians.radians
        val desiredHeading = (tagPose.rotation.z + PI).radians.standardPosition
        val headingOffset = desiredHeading.angleDistanceTo(currentRotation.standardPosition) //todo test

        //calculate horizontalVelocity (speed moving side-ways to target)
        val horizontalVelocity = -horizontalMovementPID.calculate(-swerveDrive.pose.x)

        //caluclate verticalVelocity (speed moving towards target)
        val verticalVelocity = -verticalMovementPID.calculate(-swerveDrive.pose.y)

        val translation = Translation2d(horizontalVelocity, verticalVelocity)


 // Set the desired value for the distance from the tag (Typically 0)
        // Desired rotational velocity, 0 when the rotation is within 3 degrees of the desired heading

        val angleVelocity = if (!currentRotation.angleDistanceWithin(8.0.degrees, desiredHeading))
            { (-headingOffset.asRadians *0.05).radiansPerSecond }
        else { 0.0.radiansPerSecond }

        speedConsumer(
            Transform2d(
                translation,
                Rotation2d(angleVelocity.asRadiansPerSecond)
            )
        )
            SmartDashboard.putNumber("Horizontal Velocity", translation.y)
            SmartDashboard.putNumber("Vertical Velocity", translation.x)
            SmartDashboard.putNumber("Tag Heading", tagPose.rotation.z)
            SmartDashboard.putNumber("Deired Heading", desiredHeading.asDegrees)
            SmartDashboard.putNumber("currentHeading", currentRotation.standardPosition.asDegrees)
            SmartDashboard.putNumber("angle Velocity", angleVelocity.asRotationsPerSecond)

    }
    //automatically finds the face of the reef that robot is closest to and returns the ID of the apriltag on that face of the reef
    fun idAutoSelect(robotPose: Pose2d, alliance: Alliance): Int{
        var reefPose = Pose2d()
        var tags = mutableListOf<Int>()
        if (alliance == Alliance.Red){
            reefPose = Pose2d(Translation2d(13.06185,4.03), Rotation2d())
            tags = mutableListOf(8, 9, 10, 11, 6, 7)

        } else if(alliance == Alliance.Blue){
            reefPose = Pose2d(Translation2d(0.0,0.0), Rotation2d()) //TODO
            tags = mutableListOf(20, 19, 18, 17, 22, 21)
        }
        var currentAngle = atan2(-robotPose.x - reefPose.x, -robotPose.y - reefPose.y).radiansToDegrees()
        if (currentAngle<0){
            currentAngle += 360
        }
        SmartDashboard.putNumber("currentAngle", currentAngle)
        when{
            currentAngle.within(30.0, 30.0) -> return tags[0]
            currentAngle.within(30.0, 90.0) -> return tags[5]
            currentAngle.within(30.0, 150.0) -> return tags[4]
            currentAngle.within(30.0, 210.0) -> return tags[3]
            currentAngle.within(30.0, 270.0) -> return tags[2]
            currentAngle.within(15.0, 330.0) -> return tags[1]
        }
        return tags[0]
    }
    //calculate distance between two angles, works with 0-360 angle modulus
    fun angleDist(a: AngleUnit, b: AngleUnit): AngleUnit {
        val diff = b - a;
        if (diff.asDegrees <= 180.0) return diff;
        else return ((diff.asDegrees % 180.0) - 180.0).degrees;
    }

    /**converts other angle modulus formats to 0-360 format*/
    fun realMod(a: AngleUnit, b: AngleUnit): AngleUnit {
        val mod = a % b;
        if (mod.asRadians >= 0) return mod;
        else return mod + b;
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        SmartDashboard.putNumber("horizontalVelocity", 0.0)
        SmartDashboard.putNumber("verticalVelocity", 0.0)
        speedConsumer( Transform2d(0.0, 0.0, Rotation2d(0.0) ) )
    }






}
