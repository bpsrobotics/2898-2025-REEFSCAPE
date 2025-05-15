package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import frc.robot.Constants.AutoConstants.RotationD
import frc.robot.Constants.AutoConstants.RotationI
import frc.robot.Constants.AutoConstants.RotationP
import frc.robot.Constants.AutoConstants.TranslationD
import frc.robot.Constants.AutoConstants.TranslationI
import frc.robot.Constants.AutoConstants.TranslationP
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Drivetrain.driveConsumer
import frc.robot.subsystems.Drivetrain.getAlliance

object Autos {

    init {
        AutoBuilder.configure(
            Drivetrain::getPose,  // Robot pose supplier
            Drivetrain::resetOdometry,  // Method to reset odometry (will be called if your auto has a starting pose)
            Drivetrain::getRobotVelocity,  // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            driveConsumer,  // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            PPHolonomicDriveController( // PPolonomicController is the built-in path following controller for holonomic drive trains
                PIDConstants(TranslationP, TranslationI, TranslationD),  // Translation PID constants
                PIDConstants(RotationP, RotationI, RotationD)
            ),
            Constants.AutoConstants.Robot_Config,
            getAlliance,
            Drivetrain// Reference to this subsystem to set requirements
        )
    }


}