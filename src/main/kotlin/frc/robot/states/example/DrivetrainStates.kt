package frc.robot.states.example

import edu.wpi.first.wpilibj2.command.button.Trigger
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

enum class DrivetrainStates {
    DRIVING,
    IDLE,
    LOCKED,
    PATHFINDING,
    ALIGNING;

    val trigger = Trigger { state == this }

    fun set() {
        state = this
    }
}

@LoggedOutput(LogLevel.COMP)private var state: DrivetrainStates = DrivetrainStates.IDLE

// Example Usage
fun example() {
    DrivetrainStates.DRIVING.set()
}
