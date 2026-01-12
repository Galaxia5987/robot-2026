package frc.robot.subsystems.drive

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.deg

const val DEFAULT_CONTROLLER_NAME = "RegularAlign"
const val LINEAR_KP = 5.0
const val LINEAR_KI = 0.0
const val LINEAR_KD = 0.0

const val ANGULAR_KP = 10.0
const val ANGULAR_KI = 0.0
const val ANGULAR_KD = 0.0

val MAX_ANGULAR_VELOCITY: AngularVelocity = DegreesPerSecond.of(360.0)
val MAX_ANGULAR_ACCELERATION: AngularAcceleration =
    DegreesPerSecondPerSecond.of(280.0)

val ANGULAR_CONSTRAINTS =
    TrapezoidProfile.Constraints(
        MAX_ANGULAR_VELOCITY.`in`(RadiansPerSecond),
        MAX_ANGULAR_ACCELERATION.`in`(RadiansPerSecondPerSecond)
    )

val TOLERANCE = Pose2d(3.cm, 3.cm, Rotation2d(3.deg))
