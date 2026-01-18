package frc.robot.subsystems.shooter.turret

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.lib.convertTo360
import frc.robot.lib.extensions.distanceFromPoint
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.rot
import frc.robot.lib.extensions.rotationToPoint
import frc.robot.lib.wrapAround
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

val HUB_LOCATION = Translation2d(4572.mm, 7632.7.mm)
const val HUB_PATH = "Subsystem/Hub"

@LoggedOutput(LogLevel.COMP, path = HUB_PATH)
val robotDistanceFromHub
    get() = drive.localPose.distanceFromPoint(HUB_LOCATION)

@LoggedOutput(LogLevel.DEV, path = HUB_PATH)
val angleFromRobotToHub
    get() = (drive.localPose.translation.rotationToPoint(HUB_LOCATION))

@LoggedOutput(LogLevel.DEV, path = HUB_PATH)
// +180 degrees since the turret's zero angle is exactly opposite of the swerve's zero angle.
val turretToRobotHubAngle: Rotation2d
    get() =
        (-angleFromRobotToHub + Rotation2d.k180deg + drive.localPose.rotation)
            .convertTo360()

val turretAngleToHub: Angle
    get() =
        turretToRobotHubAngle.measure
            .wrapAround(
                SOFTWARE_LIMIT_CONFIG.ReverseSoftLimitThreshold.rot,
                SOFTWARE_LIMIT_CONFIG.ForwardSoftLimitThreshold.rot
            )

@LoggedOutput(LogLevel.COMP, path = HUB_PATH)
val isTurretAligned = Trigger {
    turretAngleToHub.isNear(
        turretToRobotHubAngle.measure,
        TOLERANCE
    )
}