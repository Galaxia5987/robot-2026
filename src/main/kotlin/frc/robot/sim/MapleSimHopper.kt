package frc.robot.sim

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.drive
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.toPose
import frc.robot.lib.extensions.toPose3d
import frc.robot.lib.extensions.toTranslation3d
import frc.robot.lib.getTranslation2d
import frc.robot.subsystems.sensors.Sensors
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
            getTranslation2d((-15).cm, (15).cm)
        )

    private fun createLayers(numberOfLayers: Int): List<Pose3d> =
        ((0 until numberOfLayers).flatMap { layer ->
            val heightOffset = (0.2 + 0.15 * layer).m

            BALL_POSES_RELATIVE_TO_ROBOT.map {
                it.toPose().toPose3d() +
                    Transform3d(
                        drive.pose.translation.toTranslation3d(heightOffset),
                        Rotation3d()
                    )
            }
        })

    val thirdFull
        get() = createLayers(1).toTypedArray()
    val empty
        get() = listOf<Pose3d>().toTypedArray()
    val halfFull
        get() = createLayers(2).toTypedArray()
    val full
        get() = createLayers(3).toTypedArray()

    var fuelInRobotPoses = empty
    fun Updatefuel(){
         val hasFuel= Sensors.hasFuel.asBoolean
        val isHalfFull= Sensors.isHalfFull.asBoolean
        val isFull= Sensors.isFull.asBoolean

        fuelInRobotPoses = when {
            //Shouldn't happen, Paulo requested for finding sensor errors
            !hasFuel && isHalfFull -> full.sliceArray(14..21)
            !hasFuel   -> empty
            isFull     -> full
            isHalfFull -> halfFull
            else       -> thirdFull
        }
    }

    val hasFuelChanged = Sensors.hasFuel.onChange(Commands.runOnce({Updatefuel()}))
    val isHalfFullChanged = Sensors.isHalfFull.onChange(Commands.runOnce({Updatefuel()}))
    val isFullChanged = Sensors.isFull.onChange(Commands.runOnce({Updatefuel()}))


}
