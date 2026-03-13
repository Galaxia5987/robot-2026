package frc.robot.subsystems.shooter.turret

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Angle
import frc.robot.drive
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.rot
import frc.robot.lib.extensions.rotationToPoint
import frc.robot.lib.extensions.toTranslation3d
import frc.robot.lib.getPose3d
import frc.robot.lib.getRotation3d
import frc.robot.states.setpoints_manager.SetpointsManager.currentGoal

const val HUB_PATH = "Subsystems/Hub"

val TURRET_TO_ROBOT = Translation2d((-117.5).mm, 207.5.mm)

val turretTranslationFieldOriented: Translation2d
    get() =
        drive.pose.translation.plus(
            TURRET_TO_ROBOT.rotateBy(drive.pose.rotation)
        )

val compensatedTurretTranslationFieldOriented: Translation2d
    get() =
        drive.compensatedPose.translation.plus(
            TURRET_TO_ROBOT.rotateBy(drive.pose.rotation)
        )

// For debugging
val turretPose
    get() =
        getPose3d(
            turretTranslationFieldOriented.toTranslation3d(50.cm),
            getRotation3d(pitch = (-90).deg)
        )

val angleFromRobotToHub: Rotation2d
    get() =
        turretTranslationFieldOriented.rotationToPoint(currentGoal.translation)

val turretAngleToHub: Angle
    get() = (drive.pose.rotation - angleFromRobotToHub).measure

val turretAimingSetpoint: Angle
    get() = constraintTurretLimits(turretAngleToHub)

fun constraintTurretLimits(angle: Angle): Angle {
    var newAngle = angle
    if (newAngle < REVERSE_LIMIT) {
        newAngle = 1.rot + newAngle
    }
    return newAngle
}
