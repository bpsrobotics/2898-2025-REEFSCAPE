
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command


class pathfinder:
    //Global constraints for all pathfinder operations.
    var constraints: PathConstraints

    //Set constraints given pathConstraints
    fun setConstraints(constraints: pathConstraints):
            this.constraints = constraints

    //A function that returns constraints
    fun getConstraints():
        return this.constraints

    //Load pathfinding command
    fun getPathTo(pos: Pose2d, goalEndVel: double = 0, goalEndRotation: double = 0)
        var pathfindingCommand: edu.wpi.first.wpilibj2.command.Command = AutoBuilder.pathfindToPose(
            targetPose,
            this.constraints,
            goalEndVel,  // Goal end velocity in meters/sec
            goalEndRotation // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        )