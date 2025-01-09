package com.team2898.robot.commands.bintake

import com.team2898.robot.subsystems.ToteManipulator
import edu.wpi.first.wpilibj2.command.Command

class Grasp : Command() {

    private val grabber = ToteManipulator
    init {
        addRequirements(ToteManipulator)
    }

    override fun execute() {
        grabber.toggle()
    }

    override fun isFinished(): Boolean {
        return false
    }

}