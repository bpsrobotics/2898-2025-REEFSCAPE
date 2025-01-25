// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems


import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.util.DriveFeedforwards
import com.pathplanner.lib.util.GeometryUtil
import edu.wpi.first.math.*
import frc.robot.Constants
import frc.robot.Constants.AutoConstants.RotationD
import frc.robot.Constants.AutoConstants.RotationI
import frc.robot.Constants.AutoConstants.RotationP
import frc.robot.Constants.AutoConstants.TranslationD
import frc.robot.Constants.AutoConstants.TranslationI
import frc.robot.Constants.AutoConstants.TranslationP
import frc.robot.OI.translationX
import frc.robot.OI.translationY
import frc.robot.OI.turnX
import frc.robot.OI.turnY
import frc.robot.subsystems.Drivetrain.run
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import swervelib.SwerveDrive
import swervelib.SwerveDriveTest
import swervelib.SwerveModule
import swervelib.math.SwerveMath
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity
import java.util.*
import java.util.function.BooleanSupplier


object  Drivetrain : SubsystemBase() {
    var swerveDrive: SwerveDrive
    private val visionDriveTest = false

    /** The maximum speed of the swerve drive */
    var maximumSpeed: Double = Units.feetToMeters(14.5)

    /** SwerveModuleStates publisher for swerve display */
    var swerveStates: StructArrayPublisher<SwerveModuleState> = NetworkTableInstance.getDefault().
    getStructArrayTopic("SwerveStates/swerveStates", SwerveModuleState.struct).publish()
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
//    lateinit var config : RobotConfig;
    init {
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH
//        try{
//            config = RobotConfig.fromGUISettings();
//        } catch (e : Exception) {
//            // Handle exception as needed
//            e.printStackTrace();
//        }
        try {
            swerveDrive =
                SwerveParser(Constants.DriveConstants.DRIVE_CONFIG).createSwerveDrive(Constants.DriveConstants.MaxSpeedMetersPerSecond)
        } catch (e: Exception){
            e.printStackTrace()
            throw RuntimeException("error creating swerve",e)
        }
        swerveDrive.setHeadingCorrection(false) // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(false) //!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.

        if (visionDriveTest) {
//                setupPhotonVision()
            // Stop the odometry thread if we are using vision that way we can synchronize updates better.
            swerveDrive.stopOdometryThread()
        }
//        setupPathPlanner()

        swerveDrive.setVisionMeasurementStdDevs(Vision.getStandardDev())
        // Updates odometry whenever a new
        Vision.listeners.add ( "UpdateOdometry") {
            val position: Pose2d = Vision.getRobotPosition(it)?.toPose2d() ?: return@add
            swerveDrive.addVisionMeasurement(position, it.timestampSeconds)
            SmartDashboard.putNumberArray("odometry/visionTranslation", doubleArrayOf(position.x, position.y))
            SmartDashboard.putNumber("odometry/visionRotation", position.rotation.degrees)
        }

    }

//    var targetStates: StructArrayPublisher<SwerveModuleState> = NetworkTableInstance.getDefault().
//    getStructArrayTopic("SwerveStates/targetStates", SwerveModuleState.struct).publish()
//
////    var targStates = arrayOf(swerveDrive.getTargetSpeeds(-translationY, -translationX, Rotation2d(-turnX)))

    override fun periodic() {


        SmartDashboard.putNumberArray("odometry/translation", doubleArrayOf(swerveDrive.pose.x, swerveDrive.pose.y))
        SmartDashboard.putNumber("odometry/rotation", swerveDrive.pose.rotation.degrees)

        swerveStates.set(swerveDrive.states)

//        targetStates.set()
//        targetStates.set(getTargetSpeeds(-translationY, -translationX, Rotation2d(-turnX)))
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */
//    fun setupPathPlanner() {
//        AutoBuilder.configure(
//            this::getPose,  // Robot pose supplier
//            this::resetOdometry,  // Method to reset odometry (will be called if your auto has a starting pose)
//            this::getRobotVelocity,  // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
//            driveConsumer,  // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
//            PPHolonomicDriveController( // PPHolonomicController is the built-in path following controller for holonomic drive trains
//                PIDConstants(TranslationP, TranslationI, TranslationD),  // Translation PID constants
//                PIDConstants(RotationP, RotationI, RotationD)
//            ),
//            config,  // The robot configuration
//            {
//                // Boolean supplier that controls when the path will be mirrored for the red alliance
//                // This will flip the path being followed to the red side of the field.
//                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
//                val alliance = DriverStation.getAlliance()
//                if (alliance.isPresent) {
//                    return@configure alliance.get() == Alliance.Red
//                }
//                false
//            },
//             this// Reference to this subsystem to set requirements
//        )
//    }

    /**
     * Directly send voltage to the drive motors.
     * @param volts The voltage to send to the motors.
     */
    fun setRawMotorVoltage(volts: Double){
        swerveDrive.modules.forEach {
            it.driveMotor.voltage = volts
        }
    }

    /**
     * Get a SysIdRoutine for the drive motors.
     * @see SysIdRoutine
     * @return A custom SysIdRoutine for the drive motors.
     */
    fun getDriveSysIDRoutine(): SysIdRoutine {
        return SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(
                { volts: Voltage ->
                    swerveDrive.modules.forEach {
                        it.driveMotor.voltage = volts.`in`(Volt)
                    }
                },
                { log: SysIdRoutineLog ->
                    swerveDrive.modules.forEach {
                        logDriveMotor(it, log)
                    }
                },
                this
            )
        )
    }

    /**
     * Generate a full command to SysID the drive motors
     * @return A command that SysIDs the drive motors.
     */
    fun getDriveSysIDCommand(): Command {
        return SequentialCommandGroup(
            getDriveSysIDRoutine().dynamic(SysIdRoutine.Direction.kForward),
            WaitCommand(1.0),
            getDriveSysIDRoutine().dynamic(SysIdRoutine.Direction.kReverse),
            WaitCommand(1.0),
            getDriveSysIDRoutine().quasistatic(SysIdRoutine.Direction.kForward),
            WaitCommand(1.0),
            getDriveSysIDRoutine().quasistatic(SysIdRoutine.Direction.kReverse)
        )
    }

    /**
     * Logging function to easily log for SysID.
     * @see SysIdRoutineLog
     * @param module The module to log.
     * @param log The SysIdRoutineLog to log to.
     */
    private fun logDriveMotor(module: SwerveModule, log: SysIdRoutineLog){
        log.motor(module.configuration.name)
            .voltage(Volt.of(module.driveMotor.voltage))
            .linearPosition(Meters.of(module.driveMotor.position))
            .linearVelocity(MetersPerSecond.of(module.driveMotor.velocity))
    }

    /**
     * Return SysID command for drive motors from YAGSL
     * @return A command that SysIDs the drive motors.
     */
    fun sysIdDriveMotor(): Command? {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setDriveSysIdRoutine(
                SysIdRoutine.Config(),
                this,
                swerveDrive, 12.0),
            3.0, 5.0, 3.0
        )
    }

    /**
     * Return SysID command for angle motors from YAGSL
     * @return A command that SysIDs the angle1 motors.
     */
    fun sysIdAngleMotorCommand(): Command {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setAngleSysIdRoutine(
                SysIdRoutine.Config(),
                this, swerveDrive
            ),
            3.0, 5.0, 3.0
        )
    }

    /**
     * Gets a command that follows a path created in PathPlanner.
     * @param pathName The path's file name.
     * @param setOdomAtStart Whether to update the robot's odometry to the start pose of the path.
     * @return A command that follows the path.
     */
//    fun getAutonomousCommand(
//        autoName: String//,
////        setOdomAtStart: Boolean
//    ): Command {
////        var startPosition: Pose2d = Pose2d()
////        if(PathPlannerAuto.getStaringPoseFromAutoFile(autoName) == null) {
////            startPosition = PathPlannerAuto.getPathGroupFromAutoFile(autoName)[0].startingDifferentialPose
////        } else {
////            startPosition = PathPlannerAuto.getStaringPoseFromAutoFile(autoName)
////        }
////
////        if(DriverStation.getAlliance() == Optional.of(Alliance.Red)){
////            startPosition = GeometryUtil.flipFieldPose(startPosition)
////        }
////
////        if (setOdomAtStart)
////        {
////            if (startPosition != null) {
////                resetOdometry(startPosition)
////            }
////        }
//
//        // TODO: Configure path planner's AutoBuilder
//        return PathPlannerAuto(autoName)
//    }

    /**
     * Advanced drive method that translates and rotates the robot, with a custom center of rotation.
     * @param translation The desired X and Y velocity of the robot.
     * @param rotation The desired rotational velocity of the robot.
     * @param fieldOriented Whether the robot's motion should be field oriented or robot oriented.
     * @param centerOfRotation The center of rotation of the robot.
     */
    fun drive(
        translation: Translation2d,
        rotation: Double = 0.0,
        fieldOriented: Boolean = false,
        centerOfRotation: Translation2d = Translation2d()
    ) {
        swerveDrive.drive(translation, rotation, fieldOriented, false, centerOfRotation)
    }

    /**
     * Simple drive method that uses ChassisSpeeds to control the robot.
     * @param velocity The desired ChassisSpeeds of the robot
     */
    fun drive(velocity: ChassisSpeeds, fieldOriented: Boolean = false) {
        if(fieldOriented) swerveDrive.driveFieldOriented(velocity); else swerveDrive.drive(velocity)
    }

    val  driveConsumer: (ChassisSpeeds) -> Unit = { fieldSpeeds: ChassisSpeeds -> drive(fieldSpeeds) }
    /**
     * Method to get the Kinematics object of the swerve drive.
     */
    fun getKinematics() = swerveDrive.kinematics

    /**
     * Method to reset the odometry of the robot to a desired pose.
     * @param initialHolonomicPose The desired pose to reset the odometry to.
     */
    fun resetOdometry(initialHolonomicPose: Pose2d) {
        swerveDrive.resetOdometry(initialHolonomicPose)
    }

    /**
     * Method to get the current pose of the robot.
     * @return The current pose of the robot.
     */
    fun getPose() = swerveDrive.pose

    /**
     * Method to display a desired trajectory to a field2d object.
     */
    fun postTrajectory(trajectory: Trajectory) {
        swerveDrive.postTrajectory(trajectory)
    }

    /**
     * Method to zero the gyro.
     */
    fun zeroGyro() {
        swerveDrive.zeroGyro()
    }

    /**
     * Method to toggle the motor's brakes.
     * @param brake Whether to set the motor's brakes to true or false.
     */
    fun setMotorBrake(brake: Boolean) {
        swerveDrive.setMotorIdleMode(brake)
    }

    /**
     * Method to get the current heading of the robot.
     * @return The current heading of the robot.
     */
    fun getHeading() = swerveDrive.yaw

    /**
     * Method to generate a ChassisSpeeds object from a desired X, Y, and Rotational velocity.
     * @param vForward The desired forward velocity of the robot.
     * @param vSide The desired side velocity of the robot.
     * @param angle The desired rotational velocity of the robot.
     * @return The generated ChassisSpeeds object.
     */
    fun getTargetSpeeds(
        vForward: Double,
        vSide: Double,
        angle: Rotation2d
    ): ChassisSpeeds {
        return swerveDrive.swerveController.getTargetSpeeds(vForward, vSide, angle.radians, getHeading().radians, maximumSpeed)
    }

    /**
     * Method to generate a ChassisSpeeds object from a desired X, Y, and angle X and Y components.
     * @param vForward The desired forward velocity of the robot.
     * @param vSide The desired side velocity of the robot.
     * @param headingX The desired X component of the angle.
     * @param headingY The desired Y component of the angle.
     * @return The generated ChassisSpeeds object.
     */
    fun getTargetSpeeds(
        vForward: Double,
        vSide: Double,
        headingX: Double,
        headingY: Double
    ): ChassisSpeeds {
        return swerveDrive.swerveController.getTargetSpeeds(vForward, vSide, headingX, headingY, getHeading().radians, maximumSpeed)
    }

    /**
     * Method to get the current field oriented velocity of the robot.
     * @return The current field oriented velocity of the robot.
     */
    fun getFieldVelocity(): ChassisSpeeds? {
        return swerveDrive.fieldVelocity
    }

    /**
     * Method to get the current robot oriented velocity of the robot.
     * @return The current robot oriented velocity of the robot.
     */
    fun getRobotVelocity(): ChassisSpeeds? {
        return swerveDrive.robotVelocity
    }

    /**
     * Method to get the SwerveController object of the swerve drive.
     * @return The SwerveController object of the swerve drive.
     */
    fun getSwerveController() = swerveDrive.swerveController

    /**
     * Method to get the SwerveDriveConfiguration object of the swerve drive.
     * @return The SwerveDriveConfiguration object of the swerve drive.
     */
    fun getSwerveDriveConfiguration() = swerveDrive.swerveDriveConfiguration

    /**
     * Method to toggle the lock position of the swerve drive to prevent motion.
     */
    fun lock() {
        swerveDrive.lockPose()
    }

    /**
     * Method to get the current pitch of the robot.
     */
    fun getPitch() = swerveDrive.pitch

    /**
     * Add a vision measurement to the swerve drive's pose estimator.
     * @param measurement The pose measurement to add.
     * @param timestamp The timestamp of the pose measurement.
     */
    fun addVisionMeasurement(measurement: Pose2d, timestamp: Double) {
        swerveDrive.addVisionMeasurement(measurement, timestamp)
    }

    /**
     * Set the standard deviations of the vision measurements.
     * @param stdDevX The standard deviation of the X component of the vision measurements.
     * @param stdDevY The standard deviation of the Y component of the vision measurements.
     * @param stdDevTheta The standard deviation of the rotational component of the vision measurements.
     */
    fun setVisionMeasurementStdDevs(stdDevX: Double, stdDevY: Double, stdDevTheta: Double) {
        swerveDrive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(stdDevX, stdDevY, stdDevTheta))
    }

    /**
     * PID Controller for the heading of the robot.
     * Currently not used, as feedforward is a much better solution to fix drift.
     */
    val HeadingPID: PIDController = PIDController(0.005, 0.01, 0.0)


    /**
     * Calculate the heading PID for the robot.
     * @param measurement The current heading of the robot.
     * @param setpoint The desired heading of the robot.
     * @return The calculated output of the PID controller.
     */
    @Deprecated("Use feedforward instead")
    fun calculateHeadingPID(measurement: Double, setpoint: Double): Double {
        return HeadingPID.calculate(measurement, setpoint)
    }

    fun driveCommand(): Command {
        return run{
            val scaledInputs = SwerveMath.scaleTranslation(Translation2d(
                MathUtil.applyDeadband(translationY, 0.1),
                MathUtil.applyDeadband(translationX, 0.1)),
                0.8
            )
            drive(
                swerveDrive.swerveController.getTargetSpeeds(scaledInputs.x, scaledInputs.y,
                MathUtil.applyDeadband(turnX, 0.1),
                MathUtil.applyDeadband(turnY, 0.1),
                swerveDrive.odometryHeading.radians,
                swerveDrive.maximumChassisVelocity //TODO Make sure that this is MaximumVelocy
            ));
        }
    }

}