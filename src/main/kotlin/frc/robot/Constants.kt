// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
@file:Suppress("unused")

package frc.robot

import beaverlib.utils.Units.Linear.feet
import beaverlib.utils.Units.Linear.feetPerSecond
import beaverlib.utils.Units.Linear.inches
import beaverlib.utils.Units.lb
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.RobotConfig
import beaverlib.utils.Units.Electrical.Current
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.util.Color
import frc.robot.Constants.DriveConstants.DriveKinematics
import frc.robot.Constants.DriveConstants.MaxSpeedMetersPerSecond
import java.io.File
import kotlin.math.PI

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 *
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
@Suppress("MemberVisibilityCanBePrivate")
class Constants {


    object DriveConstants {
        val MaxSpeedMetersPerSecond = (15.1).feetPerSecond.asMetersPerSecond
        // Chassis configuration (left to right dist of center of the wheels)
        val TrackWidth = Units.inchesToMeters(11.5)

        // Distance between centers of right and left wheels on robot (front to back dist)
        val WheelBase = Units.inchesToMeters(11.5)

        // Distance between front and back wheels on robot: CHANGE TO MATCH WITH ROBOT
        val DriveKinematics = arrayOf(
            Translation2d(WheelBase / 2, TrackWidth / 2),
            Translation2d(WheelBase / 2, -TrackWidth / 2),
            Translation2d(-WheelBase / 2, TrackWidth / 2),
            Translation2d(-WheelBase / 2, -TrackWidth / 2)
        )
        // YAGSL `File` Configs
        val DRIVE_CONFIG: File = File(Filesystem.getDeployDirectory(), "swerve1")

        val MomentOfInertia = 4.09149392  // kg * m^2

    }

    object ElevatorConstants {
        const val MaxVel = 1.0
        const val MaxAccel = 1.0

        //PID constants
        const val kP = 5.0
        const val kI = 0.0
        const val kD = 1.0

        //FF constants
        const val kS = 0.045
        const val kV = 7.44
        // 6.0
//            1.5136
        const val kG = 0.355
        const val kA = 0.0

        //Max elev driver outputs
        const val NEG_MAX_OUTPUT = -2.0
        const val POS_MAX_OUTPUT = 5.0

        //SOFT Stop limits
        const val UPPER_LIMIT = 1.45
        const val LOWER_LIMIT = 0.0

        enum class ElevatorState(val position: Double) {
            Stow(0.0),
            L2(0.21),
            L3(0.64),
            L4(1.425),
            A1(0.45),
            A2(0.9),
        }
    }

    object PivotConstants {
        //Max elev driver outputs
        const val NEG_MAX_OUTPUT = -1.75
        const val POS_MAX_OUTPUT = 2.0

        const val ObstructionAngle = 1.5

        const val kP = 6.0
        const val kI = 0.0
        const val kD = 0.5

        const val kS = 0.11
        const val kG = 0.51
        const val kV = 0.64
        // 0.84

        const val Max_Velocity = PI
        const val Max_Accel = PI
        //SOFT Stop limits
        const val UPPER_LIMIT = 0.0
        const val LOWER_LIMIT = 0.0


        // FIXME set to real positions later
        enum class PivotState(val position: Double) {
            Traverse(0.87),
            Stow(1.78),
            AngleBranch(1.4),
            VerticalBranch(0.6),
            Algae(-0.95);

            fun coralExtend() = when (this) {
                Stow -> AngleBranch
                AngleBranch -> Traverse
                Traverse -> VerticalBranch
                VerticalBranch -> VerticalBranch
                Algae -> Algae
            }
            fun coralRetract() = when (this) {
                Algae -> Algae
                VerticalBranch -> Traverse
                Traverse -> AngleBranch
                AngleBranch -> Stow
                Stow -> Stow
            }
            fun algaeExtend() = when (this) {
                Stow -> Traverse
                Traverse -> Algae
                Algae -> Algae
                VerticalBranch -> Algae
                AngleBranch -> Traverse
            }
            fun algaeRetract() = when (this) {
                Algae -> Traverse
                Traverse -> Traverse
                Stow -> Stow
                VerticalBranch -> VerticalBranch
                AngleBranch -> AngleBranch
            }
         }
    }

    object IntakeConstants {
        const val ks = 0.0
        const val kv = 0.0
        const val ka = 0.0
        const val STOP_BUFFER = 1.0
        const val CURRENT_WHEN_ROBOT_HAS_CORAL = 7.0 //FIXME set to real value
        val CORAL_COLOR = Color(255, 255, 255) //FIXME set to real value
        const val CORAL_COLOR_TOLERANCE = 10.0 //FIXME set to real value
        const val INTAKE = 0.8
        const val OUTTAKE = -0.4
    }


    object OIConstants {
        const val DriverControllerPort = 0
        @Suppress("SpellCheckingInspection")
        const val DriveDeadband = 0.05
        const val SpeedMultiplierMin = 0.4
        const val SpeedMultiplierMax = 1.0
        const val DEADZONE_THRESHOLD = 0.1
    }

    object AutoConstants {
        val Robot_Config = RobotConfig(
            (120.0).lb.asKilograms,
            MaxSpeedMetersPerSecond,
            ModuleConfig(
                (2.0).inches.asMeters,
                MaxSpeedMetersPerSecond,
                1.54,
                DCMotor.getNEO(1).withReduction(6.75),
                30.0,
                1
            ),
            *DriveKinematics

        )
        const val MaxAccelerationMetersPerSecondSquared = 3.0
        const val MaxAngularSpeedRadiansPerSecond = Math.PI
        const val MaxAngularSpeedRadiansPerSecondSquared = Math.PI
        const val PXController = 1.0
        const val PYController = 1.0
        const val PThetaController = 1.0

        const val TranslationP = 5.0
        const val TranslationI = 0.0
        const val TranslationD = 0.0

        const val RotationP = 0.01
        const val RotationI = 0.0
        const val RotationD = 0.0

        // Constraint for the motion profiled robot angle controller
        val ThetaControllerConstraints = TrapezoidProfile.Constraints(
                MaxAngularSpeedRadiansPerSecond, MaxAngularSpeedRadiansPerSecondSquared
        )
    }



    // set to operator/driver's preferences
    object ButtonConstants {
        const val CLIMBER_UP = 2 // very hard to press accidentally
        const val CLIMBER_WAIT_DURATION = 0.5

        //Driver buttons
        const val RESET_GYRO = 6

        //Operator Controls
        const val TOGGLE_STATE = 1
        const val AUTO_INTAKE = 2

        const val PIVOT_FW = 5
        const val PIVOT_BW = 3

        const val ELEV_FW = 6
        const val ELEV_BW = 4

        const val BASE_STAGE = 7
        const val CORAL_L2 = 8
        const val CORAL_L3 = 10
        const val CORAL_L4 = 12

        const val ALGAE_B1 = 9
        const val ALGAE_B2 = 11

        const val PRESS_ACTIVATE_DURATION = 0.1
        const val INPUT_BUFFER_DURATION = 0.2
    }

    object VisionConstants {
        const val CORAL_OFFSET_FROM_CENTER = 0.1524
    }



}