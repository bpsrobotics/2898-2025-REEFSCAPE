package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants.AutoConstants.RotationD
import frc.robot.Constants.AutoConstants.RotationI
import frc.robot.Constants.AutoConstants.RotationP
import frc.robot.Constants.AutoConstants.TranslationD
import frc.robot.Constants.AutoConstants.TranslationI
import frc.robot.Constants.AutoConstants.TranslationP
import frc.robot.commands.elevator.MoveElevator
import frc.robot.commands.elevator.StabilizeElevator
import frc.robot.commands.intake.RunIntake
import frc.robot.commands.intake.RunOuttake
import frc.robot.commands.sequence.PositionL2
import frc.robot.commands.sequence.Stow
import frc.robot.commands.wrist.MoveWrist
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
        NamedCommands.registerCommand("coralouttake", RunOuttake(0.8)) //Todo set this properly
        //todo NamedCommands.registerCommand("algaeintake", AlgaeIntakeOutake())

        NamedCommands.registerCommand("L1", Stow())

        NamedCommands.registerCommand("L2", PositionL2())

        NamedCommands.registerCommand("L3", MoveElevator(Constants.ElevatorConstants.ElevatorState.L3.position))

        NamedCommands.registerCommand("L4", SequentialCommandGroup(
            MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
            MoveElevator(Constants.ElevatorConstants.ElevatorState.L4.position),
            MoveWrist(Constants.PivotConstants.PivotState.VerticalBranch.position)
        ))

        NamedCommands.registerCommand("PlaceSequence", SequentialCommandGroup(
            MoveWrist(Constants.PivotConstants.PivotState.Traverse.position),
            MoveElevator(Constants.ElevatorConstants.ElevatorState.L4.position),
            MoveWrist(Constants.PivotConstants.PivotState.VerticalBranch.position),
            RunOuttake(0.8, 0.5)
        ))

        NamedCommands.registerCommand("autointake", RunIntake())

        NamedCommands.registerCommand("GetCoralStationPiece", SequentialCommandGroup(
            MoveElevator(Constants.ElevatorConstants.ElevatorState.Stow.position),
            MoveWrist(Constants.PivotConstants.PivotState.Stow.position),
            RunIntake()
        ))

        NamedCommands.registerCommand("stabilize", StabilizeElevator())
    }


}