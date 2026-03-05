package frc.robot.subsystems.shooter.turret

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.calculateYaw
import frc.robot.drive
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.rot
import frc.robot.lib.extensions.rotationToPoint
import frc.robot.lib.extensions.toTranslation3d
import frc.robot.lib.getPose3d
import frc.robot.lib.getRotation3d
import frc.robot.states.setpoints_manager.SetpointsManager.currentGoal
import frc.robot.states.setpoints_manager.SetpointsManager.isShootingOnMove
import frc.robot.states.setpoints_manager.shooting_modes.turretDistanceFromGoal
import frc.robot.states.setpoints_manager.shooting_modes.turretOrientedChassisSpeeds
import kotlin.math.abs

const val HUB_PATH = "Subsystems/Hub"

val TURRET_TO_ROBOT = Translation2d((-117.5).mm, 207.5.mm)

val turretTranslationFieldOriented: Translation2d
    get() =
        drive.pose.translation.plus(
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

private val turretAngleToHubShootOnMove: Angle
    get() =
        turretAngleToHub -
            calculateYaw(
                    turretDistanceFromGoal[m],
                turretOrientedChassisSpeeds.x,
                    turretOrientedChassisSpeeds.y
                )
                .deg

val turretAimingSetpoint: Angle
    get() {
        var angle = if (isShootingOnMove.asBoolean) turretAngleToHubShootOnMove
        else turretAngleToHub
        if (angle < REVERSE_LIMIT) {
            angle = 1.rot + angle
        }
        return angle
    }