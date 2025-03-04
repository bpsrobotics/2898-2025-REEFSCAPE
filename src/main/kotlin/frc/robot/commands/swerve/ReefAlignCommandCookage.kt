package frc.robot.commands.swerve
import beaverlib.controls.TurningPID
import beaverlib.utils.Sugar.radiansToDegrees
import beaverlib.utils.Sugar.within
import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.asDegrees
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.geometry.HedgeHogVector2
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Engine.standardPosition
import frc.robot.subsystems.Drivetrain.swerveDrive
import frc.robot.subsystems.PathPlanner
import frc.robot.subsystems.aprilTagFieldInGame
import kotlin.math.PI
import kotlin.math.atan2


/**
 * Align with the best tag, using odometry to supplement when the camera has not seen a tag.
 * @param speedConsumer Field relative drive function
 * @param horizontalOffset How far to the left or right you want to align with the tag
 */
class ReefAlignCommandCookage(
    val speedConsumer: (Transform2d) -> Unit,
    val horizontalOffset : Double = 0.0
) : Command() {

    val turningPID = TurningPID(0.1,0.01)
    val horizontalMovementPID = PIDController(1.5, 0.0,0.05)
    val verticalMovementPID = PIDController(1.5, 0.0, 0.05)
    val xOffset = 0.5
    val yOffset = horizontalOffset
    var trackedTagID = 0
    var tagPose : Pose3d = Pose3d()
    var desiredHeading = swerveDrive.pose.rotation.radians.radians.standardPosition
    lateinit var path: PathPlannerPath
    var command : Command = Commands.none()

    override fun initialize(){
        val alliance = DriverStation.getAlliance().orElse(Alliance.Red)
        trackedTagID = idAutoSelect(swerveDrive.pose, alliance)
        tagPose = aprilTagFieldInGame.getTagPose(trackedTagID).get()
        desiredHeading = (tagPose.rotation.z + PI).radians.standardPosition


        val offset = HedgeHogVector2(xOffset, yOffset)
        val goalPosition = HedgeHogVector2(tagPose.toPose2d()) + offset.rotateBy(tagPose.rotation.z)

        val directionOfTravel = atan2(swerveDrive.pose.x - tagPose.x, swerveDrive.pose.y - tagPose.y)

//        val waypoints = PathPlannerPath.waypointsFromPoses(
//            Pose2d(swerveDrive.pose.x, swerveDrive.pose.y, Rotation2d.fromRadians(directionOfTravel)),
//            Pose2d(goalPosition.x, goalPosition.y, Rotation2d.fromRadians(directionOfTravel)),
//        )
//
//        val constraints = PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI)

        path = PathPlanner.generatePath(Pose2d(goalPosition.x, goalPosition.y, Rotation2d.fromRadians(directionOfTravel)))
        try {
            // Create a path following command using AutoBuilder. This will also trigger event markers.
            command = AutoBuilder.followPath(path)
        } catch (e: Exception){
            e.printStackTrace()
            throw RuntimeException("error creating swerve",e)
        }
    }

    override fun isFinished(): Boolean {
        return command.isFinished
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







}
