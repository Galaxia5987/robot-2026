package frc.robot.sim

import edu.wpi.first.math.geometry.Translation3d
import frc.robot.calculateVelocity
import frc.robot.drive
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.mps
import frc.robot.lib.getTranslation2d
import frc.robot.subsystems.shooter.hood.Hood
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly
import java.util.function.Supplier
import java.util.logging.Logger

class MapleSimShooter {
    private val noteOnFly =
        NoteOnFly(
            drive.pose.translation,
            getTranslation2d(0.0, 0.0),
            drive.chassisSpeeds,
            -drive.pose.rotation,
            0.0.cm,
            calculateVelocity(0.0, 0.0, 0.0).mps,
            Hood.inputs.position
        )
    private val Target =
        noteOnFly
            .withTargetPosition(
                Translation3d(4620.41.mm, 4034.63.mm, 1.82.m)
                    as Supplier<Translation3d?>?
            )
            .withTargetTolerance(Translation3d(10.0.cm, 10.cm, 10.cm))
            .withHitTargetCallBack { print("Hit hub, +67 points!") }

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
