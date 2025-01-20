package frc.robot.commands.swerve
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Vision

class CoralAlignCommand : Command() {
    private var distX = 0.0
    private var distY = 0.0
    private var yaw = 0.0
    private val runtime = Timer()
    private val swerve: Drivetrain
    private val isFinished = false

//    private fun getClosestTag(results, int: ): Int {
//        val trackedDistances = mutableListOf<Double>()
//        val trackedTags = mutableListOf<Int>()
//        for (i in results){
//            if (i.fiducialId in validTags){
//                trackedDistances.add(i.bestCameraToTarget.x)
//                trackedTags.add(i.fiducialId)
//            }
//        }
//        var minIndex = 0
//        for (i in 1 until trackedDistances.size) {
//            if (trackedDistances[i] < trackedDistances[minIndex]) {
//                minIndex = i
//            }
//        }
//        return trackedTags[minIndex]
//    }
    init {
        this.swerve = Drivetrain
    }

    override fun initialize(){
        runtime.reset()
        runtime.start()
    }

    override fun execute() {
        Vision.listeners.add("UpdateAlighCommand"){
            val trackedTarget = it.bestTarget.getBestCameraToTarget()
            distY = trackedTarget.y
            distX = trackedTarget.x
            yaw = it.bestTarget.yaw
        }


    }

    override fun isFinished(): Boolean {
        return isFinished

    }




}