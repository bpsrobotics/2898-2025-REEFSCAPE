// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
@file:Suppress("unused")

package frc.robot

import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Filesystem
import java.io.File

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
        val MaxSpeedMetersPerSecond = 4.5

        // YAGSL `File` Configs
        val DRIVE_CONFIG: File = File(Filesystem.getDeployDirectory(), "swerve")

    }

    object ElevatorConstants {
        const val MaxVel = 2.0
        const val MaxAccel = 2.0
        const val kP = 0.0
        const val kI = 0.0
        const val kD = 0.0
        const val kG = 0.4

        enum class ElevatorState(val position: Double?) {
            Custom(0.0),
            Stow(0.0),
            L2(0.1),
            L3(0.1),
            L4(0.1),
            A1(0.1),
            A2(0.1)

        }
    }

    object PivotConstants {

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

        //FIXME Operator Controls
        const val BASE_STAGE = 6
        const val CORAL_L2 = 5
        const val CORAL_L3 = 3
        const val CORAL_L4 = 11

        const val ALGAE_B1 = 8
        const val ALGAE_B2 = 9

        const val PROCESSOR = 10

        const val PRESS_ACTIVATE_DURATION = 0.1
        const val INPUT_BUFFER_DURATION = 0.2
    }



}