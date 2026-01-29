package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.sensors.Sensors.isHalfFull

val canCloseIntake = isHalfFull

val cantCloseIntake = isHalfFull.negate()

fun changeIntakingStates(): Command {
    return if (isHalfFull.asBoolean == true) {
        IntakingStates.OPEN.set()
    } else {
        IntakingStates.CLOSED.set()
    }
}
