package frc.robot.subsystems.shooter.turret

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.lib.convertTo360
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.rot
import frc.robot.lib.extensions.rotationToPoint
import frc.robot.lib.extensions.toPose
import frc.robot.lib.extensions.toTranslation3d
import frc.robot.lib.getPose3d
import frc.robot.lib.getRotation3d
import frc.robot.lib.wrapAround
import frc.robot.states.setpoints_manager.currentGoal
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

const val HUB_PATH = "Subsystems/Hub"

val TURRET_TO_ROBOT = Translation2d((-117.5).mm, 207.5.mm)

val turretTranslationFieldOriented
    get() = drive.localPose.translation.plus(TURRET_TO_ROBOT.rotateBy(drive.localPose.rotation))

@LoggedOutput(LogLevel.DEV, path = HUB_PATH)
val turretPose
    get() = getPose3d(turretTranslationFieldOriented.toTranslation3d(50.cm), getRotation3d(pitch = (-90).deg))

@LoggedOutput(LogLevel.DEV, path = HUB_PATH)
val angleFromRobotToHub
    get() = turretTranslationFieldOriented.rotationToPoint(currentGoal.translation)

@LoggedOutput(LogLevel.DEV, path = HUB_PATH)
val turretToRobotHubAngle: Rotation2d
    get() =
        (-drive.localPose.rotation + angleFromRobotToHub)


val turretAngleToHub: Angle
    get() =
        turretToRobotHubAngle.measure
            .wrapAround(
            SOFTWARE_LIMIT_CONFIG.ReverseSoftLimitThreshold.rot,
            SOFTWARE_LIMIT_CONFIG.ForwardSoftLimitThreshold.rot
       )

@LoggedOutput(LogLevel.COMP, path = HUB_PATH)
val isTurretAligned = Trigger {
    turretAngleToHub.isNear(turretAngleToHub, TOLERANCE)
}
