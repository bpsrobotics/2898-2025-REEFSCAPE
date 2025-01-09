package com.team2898.robot.commands.bintake

import com.team2898.robot.subsystems.ToteManipulator
import edu.wpi.first.wpilibj2.command.Command
import java.util.function.DoubleSupplier

class RunBintake(val speed: DoubleSupplier) : Command() {

    private val rollers = ToteManipulator

    init {
        addRequirements(ToteManipulator)
    }

    override fun execute() {
        rollers.setMotors(speed.asDouble)
    }
}