// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib;

import static frc.robot.subsystems.drive.Drive.getModuleTranslations;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.*;

// Taken from 6238 Mechanical Advantage
public class BetterPoseEstimator {
    private static final double poseBufferSizeSec = 2.0;
    private static final Matrix<N3, N1> odometryStateStdDevs =
            new Matrix<>(VecBuilder.fill(0.3, 0.3, 0.01));

    private Pose2d odometryPose = Pose2d.kZero;
    private Pose2d estimatedPose = Pose2d.kZero;

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
            TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
    private final TimeInterpolatableBuffer<Rotation3d> rotationBuffer =
            TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

    // OPTIMIZATION: Store Q std devs as primitives instead of a Matrix to avoid EJML lookups
    private final double qStdDevX;
    private final double qStdDevY;
    private final double qStdDevTheta;

    private final SwerveDriveKinematics kinematics;
    private SwerveModulePosition[] lastWheelPositions =
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private Rotation2d gyroOffset = Rotation2d.kZero;

    private ChassisSpeeds robotVelocity = new ChassisSpeeds();
    private ChassisSpeeds robotSetpointVelocity = new ChassisSpeeds();

    private static BetterPoseEstimator instance;

    public static BetterPoseEstimator getInstance() {
        if (instance == null) instance = new BetterPoseEstimator();
        return instance;
    }

    private BetterPoseEstimator() {
        qStdDevX = Math.pow(odometryStateStdDevs.get(0, 0), 2);
        qStdDevY = Math.pow(odometryStateStdDevs.get(1, 0), 2);
        qStdDevTheta = Math.pow(odometryStateStdDevs.get(2, 0), 2);

        kinematics = new SwerveDriveKinematics(getModuleTranslations());
    }

    public ChassisSpeeds getRobotVelocity() {
        return robotVelocity;
    }

    public void setRobotVelocity(ChassisSpeeds robotVelocity) {
        this.robotVelocity = robotVelocity;
    }

    public ChassisSpeeds getRobotSetpointVelocity() {
        return robotSetpointVelocity;
    }

    public void setRobotSetpointVelocity(ChassisSpeeds robotSetpointVelocity) {
        this.robotSetpointVelocity = robotSetpointVelocity;
    }

    public Pose2d getOdometryPose() {
        return odometryPose;
    }

    public void setOdometryPose(Pose2d odometryPose) {
        this.odometryPose = odometryPose;
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public void setEstimatedPose(Pose2d estimatedPose) {
        this.estimatedPose = estimatedPose;
    }

    public void resetPose(Pose2d pose) {
        gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
        estimatedPose = pose;
        odometryPose = pose;
        poseBuffer.clear();
    }

    public Rotation2d getRotation() {
        return estimatedPose.getRotation();
    }

    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
    }

    public ChassisSpeeds getFieldSetpointVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotSetpointVelocity, getRotation());
    }

    public void addOdometryObservation(OdometryObservation observation) {
        Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
        lastWheelPositions = observation.wheelPositions();
        Pose2d lastOdometryPose = odometryPose;
        odometryPose = odometryPose.exp(twist);

        if (observation.yaw() != null) {
            Rotation2d angle = observation.yaw().plus(gyroOffset);
            odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
        }

        poseBuffer.addSample(observation.timestamp(), odometryPose);

        if (observation.roll() != null
                && observation.pitch() != null
                && observation.yaw() != null) {
            rotationBuffer.addSample(
                    observation.timestamp(),
                    new Rotation3d(
                            observation.roll().getRadians(),
                            observation.pitch().getRadians(),
                            observation.yaw().getRadians()));
        }

        Twist2d finalTwist = lastOdometryPose.log(odometryPose);
        estimatedPose = estimatedPose.exp(finalTwist);
    }

    public void addVisionObservation(VisionObservation observation) {
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSec
                    > observation.timestamp()) {
                return;
            }
        } catch (NoSuchElementException ex) {
            return;
        }

        Optional<Pose2d> sampleOpt = poseBuffer.getSample(observation.timestamp());
        if (sampleOpt.isEmpty()) return;
        Pose2d sample = sampleOpt.get();

        Transform2d sampleToOdometryTransform = new Transform2d(sample, odometryPose);
        Transform2d odometryToSampleTransform = new Transform2d(odometryPose, sample);
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

        double r0 = observation.xStdDev() * observation.xStdDev();
        double r1 = observation.yStdDev() * observation.yStdDev();
        double r2 = observation.thetaStdDev() * observation.thetaStdDev();

        double k0 = (qStdDevX == 0.0) ? 0.0 : qStdDevX / (qStdDevX + Math.sqrt(qStdDevX * r0));
        double k1 = (qStdDevY == 0.0) ? 0.0 : qStdDevY / (qStdDevY + Math.sqrt(qStdDevY * r1));
        double k2 =
                (qStdDevTheta == 0.0)
                        ? 0.0
                        : qStdDevTheta / (qStdDevTheta + Math.sqrt(qStdDevTheta * r2));

        Transform2d transform =
                new Transform2d(estimateAtTime, observation.visionPose().toPose2d());

        // Apply the Kalman gain scalars directly
        Transform2d scaledTransform =
                new Transform2d(
                        transform.getX() * k0,
                        transform.getY() * k1,
                        Rotation2d.fromRadians(transform.getRotation().getRadians() * k2));

        estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
    }

    public Optional<Pose2d> getEstimatedPoseAtTimestamp(double timestamp) {
        Optional<Pose2d> oldOdometryPose = poseBuffer.getSample(timestamp);
        if (oldOdometryPose.isEmpty()) {
            return Optional.empty();
        }

        return Optional.of(
                estimatedPose.transformBy(new Transform2d(odometryPose, oldOdometryPose.get())));
    }

    public Optional<Rotation3d> getEstimatedRotation3dAtTimestamp(double timestamp) {
        return rotationBuffer.getSample(timestamp);
    }

    public record OdometryObservation(
            double timestamp,
            SwerveModulePosition[] wheelPositions,
            Rotation2d roll,
            Rotation2d pitch,
            Rotation2d yaw) {}

    public record VisionObservation(
            Pose3d visionPose,
            double timestamp,
            double xStdDev,
            double yStdDev,
            double thetaStdDev) {}
}
