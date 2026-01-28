package frc.robot.states.intaking

import frc.robot.subsystems.sensors.Sensors.isHalfFull

val canCloseIntake = isHalfFull

val cantCloseIntake = isHalfFull.negate()
