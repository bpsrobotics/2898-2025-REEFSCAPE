package frc.robot.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants.DriveConstants.MaxSpeedMetersPerSecond
import frc.robot.subsystems.Drivetrain

class TopTenAutosThatMove(val time : Double = 15.0, val speed: Double = 2.0) : Command() {
    val timer = Timer()
    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        timer.restart()
    }
    override fun execute() {
        Drivetrain.drive(ChassisSpeeds(speed * MaxSpeedMetersPerSecond, 0.0, 0.0))
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(time)
    }

}