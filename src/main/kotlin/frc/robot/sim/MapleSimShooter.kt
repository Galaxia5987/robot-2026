package frc.robot.sim

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.math.geometry.Translation3d
import frc.robot.calculateVelocity
import frc.robot.drive
import frc.robot.field.HUB_LOCATION
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.distanceFromPoint
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.mps
import frc.robot.lib.extensions.radians
import frc.robot.lib.getTranslation2d
import frc.robot.states.setpoints_manager.aimingSetpoint
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.turret.angleFromRobotToHub
import kotlinx.serialization.builtins.DoubleArraySerializer
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly
import java.util.function.Supplier
import java.util.logging.Logger

class MapleSimShooter {
    private val noteOnFly =
        NoteOnFly(
            drive.pose.translation,
            getTranslation2d(0.0, -50.0),//TODO: same here
            drive.chassisSpeeds,
            -drive.pose.rotation,
            100.0.cm,//TODO:find the actual size
            calculateVelocity(drive.pose.x, drive.pose.x, drive.pose.y).mps,
            Hood.inputs.position
        )
    private val Target =
        noteOnFly
            .withTargetPosition(
                Translation3d(4620.41.mm, 4034.63.mm, 1.82.m)
                    as Supplier<Translation3d?>?
            )
            .withTargetTolerance(Translation3d(10.0.cm, 10.cm, 10.cm))
            .withHitTargetCallBack { print("Hit hub, +68 points!") }

    private val trajectory=noteOnFly
        .withProjectileTrajectoryDisplayCallBack(
            { pose3ds ->
                    Logger.recordOutput(
                        "shooter/NoteProjectileSuccessfulShot",
                        pose3ds.toTypedArray()
                    )
                },
                { pose3ds ->
                    Logger.recordOutput(
                        "shooter/NoteProjectileUnsuccessfulShot",
                        pose3ds.toTypedArray()
                    )
            }
        )

}
