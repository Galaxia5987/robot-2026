package frc.robot.states.example

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.onTrue
import frc.robot.lib.logged_output.LoggedOutputManager
import org.littletonrobotics.junction.Logger
import org.team5987.annotation.LogLevel

enum class DrivetrainStates {
    DRIVING,
    IDLE,
    LOCKED,
    PATHFINDING,
    ALIGNING;

    val trigger = Trigger { state == this }

    fun set() {
        Logger.recordOutput("DrivertrainStates",state)
        state = this
    }
}

private var state: DrivetrainStates = DrivetrainStates.IDLE

// Example Usage
fun example() {
    DrivetrainStates.DRIVING.set()
    DrivetrainStates.DRIVING.trigger.onTrue()
}
