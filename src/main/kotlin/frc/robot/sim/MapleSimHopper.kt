package frc.robot.sim

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.toPose
import frc.robot.lib.extensions.toPose3d
import frc.robot.lib.extensions.toRotation3d
import frc.robot.lib.extensions.toTranslation3d
import frc.robot.lib.getTranslation2d
import kotlin.collections.flatMap

class MapleSimHopper {
    private val BALL_POSES_RELATIVE_TO_ROBOT =
        arrayOf(
            getTranslation2d((-5).cm, (-5).cm),
            getTranslation2d((-5).cm, (-15).cm),
            getTranslation2d(5.cm, 0.cm),
            getTranslation2d((-20).cm, (-5).cm),
            getTranslation2d(5.cm, (-15).cm),
            getTranslation2d(15.cm, 30.cm),
            getTranslation2d(15.cm, 15.cm),
            getTranslation2d(15.cm, 0.cm),
            getTranslation2d(15.cm, (-15).cm),
            getTranslation2d(15.cm, (-30).cm),
            getTranslation2d(15.cm, (0).cm),
        )

    private fun createLayers(
        numberOfLayers: Int,
        addForwardRows: Boolean = false
    ): List<Pose3d> =
        ((0 until numberOfLayers).flatMap { layer ->
            val heightOffset = (0.2 + 0.15 * layer).m

            val baseBalls =
                BALL_POSES_RELATIVE_TO_ROBOT.map {
                    (it.toPose().toPose3d() +
                            Transform3d(
                                drive.pose.translation.toTranslation3d(
                                    heightOffset
                                ),
                                Rotation3d()
                            ))
                        .rotateAround(
                            drive.pose.translation.toTranslation3d(),
                            drive.pose.rotation.toRotation3d()
                        )
                }

            if (addForwardRows) {
                val forwardRows =
                    listOf(28, 43, 57).flatMap { x ->
                        (-30..30 step 15).map { y ->
                            (getTranslation2d(x.cm, y.cm).toPose().toPose3d() +
                                    Transform3d(
                                        drive.pose.translation.toTranslation3d(
                                            heightOffset
                                        ),
                                        Rotation3d()
                                    ))
                                .rotateAround(
                                    drive.pose.translation.toTranslation3d(),
                                    drive.pose.rotation.toRotation3d()
                                )
                        }
                    }
                baseBalls + forwardRows
            } else {
                baseBalls
            }
        })

    val thirdFull
        get() = createLayers(1).toTypedArray()
    val empty
        get() = listOf<Pose3d>().toTypedArray()
    val cantCloseIntake
        get() = createLayers(2, addForwardRows = true).toTypedArray()
    val full
        get() = createLayers(3, addForwardRows = true).toTypedArray()
    var fuelInRobotPoses: () -> Array<Pose3d> = { empty }

    val emptyTrigger: Trigger = Trigger({ false })

    val thirdFullTrigger: Trigger = Trigger({ false })

    val halfFullTrigger: Trigger = Trigger({ false })

    val fullTrigger: Trigger = Trigger({ false })
}
