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
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import frc.robot.drive
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.mm
import frc.robot.lib.getRotation3d

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

private val TURRET_TRANSLATION =
    Translation3d((-117.5).mm, 207.5.mm, 360.888.mm)
val CAMERA_TO_TURRET_TRANSLATION =
    Translation3d((-75.97130).mm, 122.mm, 167.03609.mm)

// val ROBOT_TO_CAMERA: Transform3d
//    get() =
//        Pose3d(
//                (TURRET_TRANSLATION.plus(CAMERA_TO_TURRET_TRANSLATION)
//                    .rotateAround(
//                        TURRET_TRANSLATION,
//                        -Turret.wrappedPosition.toYaw()
//                    )),
//                getRotation3d(
//                    yaw = -Turret.wrappedPosition,
//                    pitch = ((-25).deg)
//                )
//            )
//            .toTransform()

val CAMERA_CONFIG: CameraConfig =
    CameraConfig(
        { Transform3d(100.mm, 100.mm, 100.mm, getRotation3d(0.deg)) },
        tagIdsToFilter = { listOf(9, 10, 26, 25) },
        stddevFactor = 1.0
    )

val OV_NAME_TO_CONFIG = mapOf("limeLight" to CAMERA_CONFIG)

var realsenseRobotToCamera = Transform3d(Translation3d(), Rotation3d())

// Basic filtering thresholds
const val MAX_AMBIGUITY = 0.3
const val MAX_Z_ERROR = 0.3

// Standard deviation baselines, for 1 meter distance and 1 tag
// (Adjusted automatically based on distance and # of tags)
const val LINEAR_STD_DEV_BASELINE = 0.03 // Meters
const val ANGULAR_STD_DEV_BASELINE = 0.08 // Radians[\]
