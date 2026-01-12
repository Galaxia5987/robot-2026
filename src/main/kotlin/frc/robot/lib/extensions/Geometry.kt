package frc.robot.lib.extensions

import com.pathplanner.lib.util.FlippingUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rectangle2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.IS_RED
import frc.robot.lib.extensions.get

fun Pose2d.moveBack(distance: Distance): Pose2d =
    this + Transform2d(-distance, Units.Meters.zero(), Rotation2d.kZero)

fun Pose2d.moveTowards(target: Pose2d, distance: Distance): Pose2d {
    val direction = this - target.withRotation(Rotation2d.kZero)
    if (direction.translation.norm < (distance[m]))
        return target.withRotation(this.rotation)
    return (this.withRotation(Rotation2d.kZero) +
            (direction) / direction.translation.norm * (-distance[m]))
        .withRotation(this.rotation)
}

fun Translation2d.getRotationToTranslation(other: Translation2d): Rotation2d =
    (this - other).angle

fun Pose2d.flip(): Pose2d = FlippingUtil.flipFieldPose(this)

fun Pose2d.flipIfNeeded(): Pose2d = if (IS_RED) this.flip() else this

fun Pose2d.withTranslation(translation: Translation2d): Pose2d =
    Pose2d(translation, this.rotation)

fun Pose2d.toPose3d(): Pose3d = Pose3d(this)

fun Pose2d.withRotation(rotation: Rotation2d): Pose2d =
    Pose2d(this.translation, rotation)

fun Pose3d.withRotation(rotation: Rotation3d): Pose3d =
    Pose3d(this.translation, rotation)

fun Pose3d.withRotation(
    roll: Double = 0.0,
    pitch: Double = 0.0,
    yaw: Double = 0.0
): Pose3d = Pose3d(this.translation, Rotation3d(roll, pitch, yaw))

fun Pose3d.withRotation(
    roll: Angle = Rotations.zero(),
    pitch: Angle = Rotations.zero(),
    yaw: Angle = Rotations.zero(),
): Pose3d = Pose3d(this.translation, Rotation3d(roll, pitch, yaw))

fun Pose2d.toTransform(): Transform2d =
    Transform2d(this.translation, this.rotation)

fun Pose2d.distanceFromPoint(translationMeters: Translation2d): Distance =
    Units.Meters.of(this.translation.getDistance(translationMeters))

fun Pose3d.toTransform(): Transform3d =
    Transform3d(this.translation, this.rotation)

fun Translation2d.flip(): Translation2d = FlippingUtil.flipFieldPosition(this)

fun Translation2d.flipIfNeeded(): Translation2d =
    if (IS_RED) this.flip() else this

fun Translation2d.toTransform(): Transform2d = Transform2d(this, Rotation2d())

fun Translation2d.toPose(): Pose2d = Pose2d(this, Rotation2d())

fun Translation2d.rotationToPoint(targetPoint: Translation2d): Rotation2d =
    Rotation2d(targetPoint.x - this.x, targetPoint.y - this.y)

fun Translation2d.rotationFromPoint(translation: Translation2d): Rotation2d =
    Rotation2d(this.x - translation.x, this.y - translation.y)

fun Rotation2d.flip(): Rotation2d = FlippingUtil.flipFieldRotation(this)

fun Rotation2d.flipIfNeeded(): Rotation2d = if (IS_RED) this.flip() else this

fun Rotation2d.toTransform(): Transform2d = Transform2d(Translation2d(), this)

fun Rotation2d.toPose(): Pose2d = Pose2d(Translation2d(), this)

fun Transform2d.toPose(): Pose2d = Pose2d(this.translation, this.rotation)

fun Transform3d.toPose(): Pose3d = Pose3d(this.translation, this.rotation)

fun Translation2d.mirror(): Translation2d =
    Translation2d(this.x, FlippingUtil.fieldSizeY - this.y)

fun Pose2d.mirror(): Pose2d = Pose2d(this.translation.mirror(), this.rotation)

fun Rectangle2d.mirror(): Rectangle2d =
    Rectangle2d(this.center.mirror(), this.xWidth, this.yWidth)

fun Rectangle2d.flip(): Rectangle2d =
    Rectangle2d(this.center.flip(), this.xWidth, this.yWidth)

fun Rectangle2d.flipIfNeeded(): Rectangle2d =
    if (IS_RED) Rectangle2d(this.center.flip(), this.xWidth, this.yWidth)
    else this
