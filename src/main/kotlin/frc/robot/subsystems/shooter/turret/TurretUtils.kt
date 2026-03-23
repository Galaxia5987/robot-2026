package frc.robot.subsystems.shooter.turret

import edu.wpi.first.units.measure.Angle
import frc.robot.lib.extensions.rot

fun constraintTurretLimits(angle: Angle): Angle {
    var newAngle = angle
    if (newAngle < REVERSE_LIMIT) {
        newAngle = 1.rot + newAngle
    }
    return newAngle
}
