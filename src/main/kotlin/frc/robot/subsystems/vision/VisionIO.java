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

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean connected = false;
        public String name = "";
        public int[] tagIds = new int[0];
        public PoseObservation estimatedPose = new PoseObservation(0.0, new Pose3d(), 0.0, 0, 0.0);
    }

    /** Represents a robot pose sample used for pose estimation. */
    record PoseObservation(
            double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance) {}

    default void updateInputs(VisionIOInputs inputs) {}
}
