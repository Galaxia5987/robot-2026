// Copyright 2021-2024 FRC 6328
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

package frc.robot.subsystems.drive.gyroIOs;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PhoenixOdometryThread;
import frc.robot.subsystems.drive.TunerConstants;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon =
            new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final StatusSignal<Angle> pitch = pigeon.getPitch();
    private final StatusSignal<Angle> roll = pigeon.getRoll();
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
    //    private final MedianFilter accelXFilter = new MedianFilter(5);
    //    private final MedianFilter accelYFilter = new MedianFilter(5);
    private final LinearFilter accelXFilter =
            LinearFilter.singlePoleIIR(0.01, 1 / Drive.ODOMETRY_FREQUENCY);
    private final LinearFilter accelYFilter =
            LinearFilter.singlePoleIIR(0.01, 1 / Drive.ODOMETRY_FREQUENCY);

    public GyroIOPigeon2() {
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
        yawVelocity.setUpdateFrequency(50.0);
        pigeon.optimizeBusUtilization();
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
    }

    @Override
    public void reset(Angle angle) {
        pigeon.setYaw(angle);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        double accelX =
                pigeon.getAccelerationX()
                        .getValue()
                        .in(edu.wpi.first.units.Units.MetersPerSecondPerSecond);
        double accelY =
                pigeon.getAccelerationY()
                        .getValue()
                        .in(edu.wpi.first.units.Units.MetersPerSecondPerSecond);
        Translation2d accel =
                new Translation2d(accelX, accelY).rotateBy(Rotation2d.fromDegrees(138.403843 + 90));
        inputs.accelerationX = edu.wpi.first.units.Units.MetersPerSecondPerSecond.of(accel.getX());
        inputs.accelerationY = edu.wpi.first.units.Units.MetersPerSecondPerSecond.of(accel.getY());
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.pitchPosition = Rotation2d.fromDegrees(pitch.getValueAsDouble());
        inputs.rollPosition = Rotation2d.fromDegrees(roll.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions =
                yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
