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

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.networktables.DoubleArrayPublisher
import edu.wpi.first.networktables.DoubleArraySubscriber
import edu.wpi.first.networktables.DoubleSubscriber
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.RobotController
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.rot
import java.util.*
import java.util.function.Supplier

/** IO implementation for real Limelight hardware. */
class VisionIOLimelight(
    name: String,
    val rotationSupplier: Supplier<Rotation2d>,
    val turretAngle: Supplier<Angle>? = null
) : VisionIO {
    private val orientationPublisher: DoubleArrayPublisher

    private val latencySubscriber: DoubleSubscriber
    private val txSubscriber: DoubleSubscriber
    private val tySubscriber: DoubleSubscriber
    private val megatag1Subscriber: DoubleArraySubscriber
    private val megatag2Subscriber: DoubleArraySubscriber

    /**
     * Creates a new VisionIOLimelight.
     *
     * @param name The configured name of the Limelight.
     * @param rotationSupplier Supplier for the current estimated rotation, used
     * for MegaTag 2.
     */
    init {
        val table = NetworkTableInstance.getDefault().getTable(name)
        orientationPublisher =
            table.getDoubleArrayTopic("robot_orientation_set").publish()
        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0)
        txSubscriber = table.getDoubleTopic("tx").subscribe(0.0)
        tySubscriber = table.getDoubleTopic("ty").subscribe(0.0)
        megatag1Subscriber =
            table
                .getDoubleArrayTopic("botpose_wpiblue")
                .subscribe(doubleArrayOf())
        megatag2Subscriber =
            table
                .getDoubleArrayTopic("botpose_orb_wpiblue")
                .subscribe(doubleArrayOf())
    }

    private fun rotateByTurret(pose: Pose3d) =
        Pose3d(
            pose
                .rotateAround(
                    Translation3d(0.mm, 0.mm, 441.837.mm),
                    Rotation3d(0.rot, 0.rot, turretAngle!!.get())
                )
                .translation,
            Rotation3d(0.rot, 0.rot, 180.0.deg - turretAngle.get())
        )

    /** Parses the 3D pose from a Limelight botpose array. */
    private fun DoubleArray.parsePose(): Pose3d {
        return Pose3d(
            this[0],
            this[1],
            this[2],
            Rotation3d(
                Math.toRadians(this[3]),
                Math.toRadians(this[4]),
                Math.toRadians(this[5])
            )
        )
    }

    override fun updateInputs(inputs: VisionIO.VisionIOInputs) {
        // Update connection status based on whether an update has been seen in the last 250ms
        inputs.connected =
            ((RobotController.getFPGATime() - latencySubscriber.lastChange) /
                1000) < 250

        // Update target observation
        inputs.latestTargetObservation =
            VisionIO.TargetObservation(
                Rotation2d.fromDegrees(txSubscriber.get()),
                Rotation2d.fromDegrees(tySubscriber.get())
            )

        // Update orientation for MegaTag 2
        orientationPublisher.accept(
            doubleArrayOf(
                rotationSupplier.get().degrees,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            )
        )
        NetworkTableInstance.getDefault()
            .flush() // Increases network traffic but recommended by Limelight

        // Read new pose observations from NetworkTables
        val tagIds: MutableSet<Int> = HashSet<Int>()
        val poseObservations: MutableList<VisionIO.PoseObservation> =
            LinkedList<VisionIO.PoseObservation>()
        mapOf(megatag1Subscriber to 1, megatag2Subscriber to 2).forEach {
            (subscriber, subNum) ->
            subscriber.readQueue().forEach { sample ->
                val rawValue = sample.value
                if (rawValue.isEmpty()) return
                for (i in 11..rawValue.size step 7) tagIds.add(
                    rawValue[i].toInt()
                )
                var pose: Pose3d = rawValue.parsePose()
                if (turretAngle != null) pose = rotateByTurret(pose)
                poseObservations.add(
                    VisionIO
                        .PoseObservation( // Timestamp, based on server timestamp of publish and
                            // latency
                            sample.timestamp * 1.0e-6 -
                                rawValue[6] * 1.0e-3, // 3D pose estimate
                            pose, // Ambiguity, zeroed because the pose is already disambiguated
                            if (subNum == 2 || rawValue.size >= 18) rawValue[17]
                            else 0.0, // Tag count
                            rawValue[7].toInt(), // Average tag distance
                            rawValue[9], // Observation type
                            if (subNum == 1)
                                VisionIO.PoseObservationType.MEGATAG_1
                            else VisionIO.PoseObservationType.MEGATAG_2
                        )
                )
            }
            // Save pose observations to inputs object
            inputs.poseObservations = poseObservations.toTypedArray()
            // Save tag IDs to inputs objects
            inputs.tagIds = tagIds.map { it }.toIntArray()
        }
    }
}
