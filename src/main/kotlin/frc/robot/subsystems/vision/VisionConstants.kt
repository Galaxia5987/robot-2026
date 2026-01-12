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
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.Millimeters
import edu.wpi.first.wpilibj.Filesystem
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.mm
import java.io.File

const val LOG_PREFIX = "Subsystems/Vision/"

// AprilTag layout
val APRILTAG_LAYOUT: AprilTagFieldLayout =
    AprilTagFieldLayout(
        File(Filesystem.getDeployDirectory(), "apriltag-locations.json")
            .toPath()
    )

// stddevFactor - Standard deviation multipliers for each camera
// (Adjust to trust some cameras more than others)
data class CameraConfig(
    val robotToCamera: Transform3d,
    val stddevFactor: Double
)

// Camera names, must match names configured on coprocessor
const val TURRET_OV_NAME = "turret"
const val LEFT_OV_NAME = "left"
const val RIGHT_OV_NAME = "right"

// Robot to camera transforms
// (Not used by Limelight, configure in web UI instead)
val TURRET_CAMERA_CONFIG: CameraConfig =
    CameraConfig(
        Transform3d(
            183.105966196.mm,
            0.mm,
            697.57321.mm,
            Rotation3d(0.deg, 13.48.deg, 180.deg)
        ),
        1.0
    )

val RIGHT: CameraConfig =
    CameraConfig(
        Transform3d(
            131.525.mm,
            (-180).mm,
            303.928.mm,
            Rotation3d(0.deg, 0.0.deg, (-90).deg)
        ),
        1.0
    )

val LEFT: CameraConfig =
    CameraConfig(
        Transform3d(
            131.525.mm,
            180.mm,
            303.928.mm,
            Rotation3d(0.deg, 0.0.deg, 90.deg)
        ),
        1.0
    )

val OV_NAME_TO_CONFIG =
    mapOf(
        TURRET_OV_NAME to TURRET_CAMERA_CONFIG,
        //    RIGHT_OV_NAME to RIGHT,
        //    LEFT_OV_NAME to LEFT,
        )

var realsenseRobotToCamera =
    Transform3d(
        Translation3d(
            Millimeters.of(330.0),
            Millimeters.of(11.326),
            Millimeters.of(-70.751)
        ),
        Rotation3d()
    )

// Basic filtering thresholds
const val MAX_AMBIGUITY = 0.3
const val MAX_Z_ERROR = 0.3

// Standard deviation baselines, for 1 meter distance and 1 tag
// (Adjusted automatically based on distance and # of tags)
const val LINEAR_STD_DEV_BASELINE = 0.03 // Meters
const val ANGULAR_STD_DEV_BASELINE = 0.08 // Radians[\]

// Multipliers to apply for MegaTag 2 observations
const val LINEAR_STD_DEV_MEGATAG2_FACTOR = 0.5 // More stable than full 3D solve
const val ANGULAR_STD_DEV_MEGATAG2_FACTOR =
    Double.POSITIVE_INFINITY // No rotation data available
