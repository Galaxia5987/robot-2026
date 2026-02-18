// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.vision.VisionIO.PoseObservation
import kotlin.math.absoluteValue
import kotlin.math.pow
import org.littletonrobotics.junction.Logger

open class Vision(
    private val consumer: VisionConsumer,
    private vararg val ios: VisionIO
) : SubsystemBase() {

    private val inputs = Array(ios.size) { VisionIOInputsAutoLogged() }
    private val disconnectedAlerts =
        Array(ios.size) { index ->
            Alert("Vision camera $index is disconnected.", AlertType.kWarning)
        }

    private fun PoseObservation.isInvalid(): Boolean =
        tagCount == 0 || // Must have at least one tag
        (tagCount == 1 &&
                ambiguity > MAX_AMBIGUITY) || // Cannot be high ambiguity
            pose.z.absoluteValue >
                MAX_Z_ERROR || // Must have realistic Z coordinate
            // Must be within the field boundaries
            !(pose.x in 0.0..APRILTAG_LAYOUT.fieldLength &&
                pose.y in 0.0..APRILTAG_LAYOUT.fieldWidth)

    private fun PoseObservation.isValid(): Boolean = !this.isInvalid()

    private fun PoseObservation.calculateStddev(): Pair<Double, Double> {
        val stdFactor = averageTagDistance.pow(2.0) / tagCount
        val linearStddev = LINEAR_STD_DEV_BASELINE * stdFactor
        val angularStddev = ANGULAR_STD_DEV_BASELINE * stdFactor
        return linearStddev to angularStddev
    }

    override fun periodic() {
        ios.zip(inputs).forEachIndexed { cameraIndex, (visionIO, cameraInputs)
            ->
            // Update IO + logging
            visionIO.updateInputs(cameraInputs)
            Logger.processInputs(
                "$LOG_PREFIX${cameraInputs.name}",
                cameraInputs
            )

            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!cameraInputs.connected)

            val estimatedPose = cameraInputs.estimatedPose
            if (estimatedPose.isInvalid()) return@forEachIndexed

            val (linearStdDev, angularStdDev) = estimatedPose.calculateStddev()

            consumer.accept(
                estimatedPose.pose.toPose2d(),
                estimatedPose.timestamp,
                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
            )
        }
    }

    fun interface VisionConsumer {
        fun accept(
            visionRobotPoseMeters: Pose2d,
            timestampSeconds: Double,
            visionMeasurementStdDevs: Matrix<N3, N1>
        )
    }
}
