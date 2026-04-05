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

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import frc.robot.drive
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.toTransform
import frc.robot.lib.extensions.toYaw
import frc.robot.lib.getRotation3d
import frc.robot.subsystems.shooter.turret.Turret

const val LOG_PREFIX = "Subsystems/Vision/"

// AprilTag layout
val APRILTAG_LAYOUT: AprilTagFieldLayout =
    AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark)

// stddevFactor - Standard deviation multipliers for each camera
// (Adjust to trust some cameras more than others)
data class CameraConfig(
    val robotToCamera: () -> Transform3d,
    val botRotation: () -> Rotation2d = { drive.gyroRotation },
    val tagIdsToFilter: () -> List<Int>,
    val stddevFactor: Double
)

// Camera names, must match names configured on coprocessor
const val TURRET_CAMERA_NAME = "turret"

const val MAX_DISTANCE_METERS = 5.6

private val TURRET_TRANSLATION =
    Translation3d((-117.5).mm, 207.5.mm, 360.888.mm)
val CAMERA_TO_TURRET_TRANSLATION =
    Translation3d((-45.19869).mm, 120.93750.mm, 177.00942.mm)

val TURRET_CAMERA_ROBOT_TO_CAMERA: Transform3d
    get() =
        Pose3d(
                (TURRET_TRANSLATION.plus(CAMERA_TO_TURRET_TRANSLATION)
                    .rotateAround(
                        TURRET_TRANSLATION,
                        -Turret.position.toYaw()
                    )),
                getRotation3d(
                    yaw = -Turret.position,
                    pitch = ((-24.781651).deg)
                )
            )
            .toTransform()

val TURRET_CONFIG =
    CameraConfig(
        robotToCamera = { TURRET_CAMERA_ROBOT_TO_CAMERA },
        tagIdsToFilter = { listOf(9, 10, 26, 25) },
        stddevFactor = 1.0
    )

val LEFT_CONFIG =
    CameraConfig(
        robotToCamera = {
            Transform3d(
                (-59.348).mm,
                361.75150.mm,
                195.738.mm,
                Rotation3d(0.0.deg, (-20).deg, 60.deg)
            )
        },
        tagIdsToFilter = { listOf() },
        stddevFactor = 1.0
    )

val RIGHT_CONFIG =
    CameraConfig(
        robotToCamera = {
            Transform3d(
                (-237.60471).mm,
                (-373.33729).mm,
                473.25917.mm,
                Rotation3d(0.0.deg, 0.deg, (-45).deg)
            )
        },
        tagIdsToFilter = { listOf() },
        stddevFactor = 1.0
    )

val BACK_CONFIG =
    CameraConfig(
        robotToCamera = {
            Transform3d(
                (-296.15884).mm,
                17.02.mm,
                394.10380.mm,
                Rotation3d(0.0.deg, (-27.5).deg, 180.deg)
            )
        },
        tagIdsToFilter = { listOf() },
        stddevFactor = 1.0
    )

val FRONT_CONFIG =
    CameraConfig(
        robotToCamera = {
            Transform3d(
                (-174.327).mm,
                (-309.901).mm,
                528.733.mm,
                Rotation3d(0.0.deg, (-20).deg, 0.deg)
            )
        },
        tagIdsToFilter = { listOf() },
        stddevFactor = 1.0
    )

val OV_NAME_TO_CONFIG =
    mapOf<String, CameraConfig>(
        TURRET_CAMERA_NAME to TURRET_CONFIG,
        "left" to LEFT_CONFIG,
        "right" to RIGHT_CONFIG,
        "front" to FRONT_CONFIG,
        "back" to BACK_CONFIG,
    )

var realsenseRobotToCamera = Transform3d(Translation3d(), Rotation3d())

// Basic filtering thresholds
const val MAX_AMBIGUITY = 0.2
const val MAX_Z_ERROR = 0.3

// Standard deviation baselines, for 1 meter distance and 1 tag
// (Adjusted automatically based on distance and # of tags)
const val LINEAR_STD_DEV_BASELINE = 0.03 // Meters
const val ANGULAR_STD_DEV_BASELINE = 0.08 // Radians[\]
