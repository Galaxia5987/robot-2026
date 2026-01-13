package frc.robot.lib.math

import edu.wpi.first.math.Nat
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.meters
import java.util.ArrayDeque

class Translation2dMovingAverage(private val windowSize: Int) {
    private val buffer = ArrayDeque<Vector<N2>>()
    private var sum = Vector(Nat.N2())

    fun add(pose: Translation2d): Translation2d {
        val value: Vector<N2> = pose.toVector()
        sum = sum.plus(value)
        buffer.addLast(value)

        if (buffer.size > windowSize) {
            val oldest: Vector<N2> = buffer.removeFirst()
            sum = sum.minus(oldest)
        }

        val poseMatrix = sum.div(buffer.size)
        return Translation2d(poseMatrix)
    }

    fun isFull(): Boolean = buffer.size == windowSize
}

class Translation2dExponentialAverage(private val alpha: Double = 0.3) {
    private var average: Translation2d? = null

    fun add(pose: Translation2d): Translation2d {
        average =
            if (average == null) {
                pose
            } else {
                Translation2d(
                    average!!.x * (1 - alpha) + pose.x * alpha,
                    average!!.y * (1 - alpha) + pose.y * alpha
                )
            }
        return average!!
    }
}

class Translation2dMultiStabilizer(
    private val windowSize: Int = 5,
    private val maxDistanceDelta: Distance = 0.5.meters
) {
    private var filter = Translation2dMovingAverage(windowSize)
    private var lastEstimatedMeasurement: Translation2d? = null

    fun addMeasurement(translation: Translation2d): Translation2d {
        if (lastEstimatedMeasurement == null) {
            lastEstimatedMeasurement = filter.add(translation)
            return lastEstimatedMeasurement!!
        }

        if (
            lastEstimatedMeasurement!!.getDistance(translation).m >
                maxDistanceDelta
        ) {
            filter = Translation2dMovingAverage(windowSize)
        }

        lastEstimatedMeasurement = filter.add(translation)
        return lastEstimatedMeasurement!!
    }
}
