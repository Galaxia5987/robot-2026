package frc.robot.states.example

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

// Mock Subsystem Command
fun drive(): Command {
    return Commands.none()
}

val driveTrigger = DrivetrainStates.DRIVING.trigger.whileTrue(drive())
