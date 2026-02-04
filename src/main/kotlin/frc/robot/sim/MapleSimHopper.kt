package frc.robot.sim

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.drive
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.toPose
import frc.robot.lib.extensions.toPose3d
import frc.robot.lib.extensions.toTranslation3d
import frc.robot.lib.getTranslation2d
import frc.robot.subsystems.sensors.Sensors
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import kotlin.collections.flatMap

class MapleSimHopper {
    private val BALL_POSES_RELATIVE_TO_ROBOT = arrayOf(
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
        getTranslation2d((-15).cm, (15).cm)
    )


    private fun createLayers(numberOfLayers: Int): List<Pose3d> =
        ((0 until numberOfLayers).flatMap { layer ->
            val heightOffset = (0.2 + 0.15 * layer).m

            BALL_POSES_RELATIVE_TO_ROBOT.map {
                it.toPose().toPose3d() + Transform3d(
                    drive.pose.translation.toTranslation3d(heightOffset),
                    Rotation3d()
                )
            }
        })

    val empty = listOf<Pose3d>()
    val thirdFull = createLayers(1)
    val halfFull = createLayers(2)
    val full = createLayers(3)

    val setEmpty = Sensors.hasFuel.negate().onTrue(Commands.runOnce({fuelInRobotPoses = empty}))
    val setThirdFull = Sensors.hasFuel.and(Sensors.isHalfFull.negate()).onTrue(Commands.runOnce({fuelInRobotPoses = thirdFull}))
    val setHalfFull = Sensors.isHalfFull.negate().and(Sensors.isFull.negate()).onTrue(Commands.runOnce({fuelInRobotPoses = halfFull}))
    val setFull = Sensors.isFull.onTrue(Commands.runOnce({fuelInRobotPoses = full}))

    var fuelInRobotPoses = empty
}