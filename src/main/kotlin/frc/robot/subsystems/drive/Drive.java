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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ConstantsKt;
import frc.robot.autonomous.AutoCommandsKt;
import frc.robot.lib.BetterPoseEstimator;
import frc.robot.lib.LocalADStarAK;
import frc.robot.lib.Mode;
import frc.robot.lib.sysid.SysIdable;
import frc.robot.subsystems.drive.ModuleIOs.Module;
import frc.robot.subsystems.drive.ModuleIOs.ModuleIO;
import frc.robot.subsystems.drive.gyroIOs.GyroIO;
import frc.robot.subsystems.drive.gyroIOs.GyroIOInputsAutoLogged;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import kotlin.Unit;
import kotlin.jvm.functions.Function1;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Drive extends SubsystemBase implements SysIdable {
    // TunerConstants doesn't include these constants, so they are declared locally
    public static final double ODOMETRY_FREQUENCY =
            new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
    public static final double DRIVE_BASE_RADIUS =
            Math.max(
                    Math.max(
                            Math.hypot(
                                    TunerConstants.FrontLeft.LocationX,
                                    TunerConstants.FrontLeft.LocationY),
                            Math.hypot(
                                    TunerConstants.FrontRight.LocationX,
                                    TunerConstants.FrontRight.LocationY)),
                    Math.max(
                            Math.hypot(
                                    TunerConstants.BackLeft.LocationX,
                                    TunerConstants.BackLeft.LocationY),
                            Math.hypot(
                                    TunerConstants.BackRight.LocationX,
                                    TunerConstants.BackRight.LocationY)));

    private static final double compensationConstant = 0.0; // Sec

    // PathPlanner config constants
    private static final double ROBOT_MASS_KG = 67.15;
    private static final double ROBOT_MOI = 8.132;
    private static final double WHEEL_COF = 1.0;
    private static final RobotConfig PP_CONFIG =
            new RobotConfig(
                    ROBOT_MASS_KG,
                    ROBOT_MOI,
                    new ModuleConfig(
                            TunerConstants.FrontLeft.WheelRadius,
                            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                            WHEEL_COF,
                            DCMotor.getKrakenX60Foc(1)
                                    .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                            TunerConstants.FrontLeft.SlipCurrent,
                            1),
                    getModuleTranslations());

    public static final DriveTrainSimulationConfig mapleSimConfig =
            DriveTrainSimulationConfig.Default()
                    .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
                    .withCustomModuleTranslations(getModuleTranslations())
                    .withGyro(COTS.ofPigeon2())
                    .withSwerveModule(
                            new SwerveModuleSimulationConfig(
                                    DCMotor.getKrakenX60(1),
                                    DCMotor.getKrakenX44Foc(1),
                                    TunerConstants.FrontLeft.DriveMotorGearRatio,
                                    TunerConstants.FrontLeft.SteerMotorGearRatio,
                                    Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                                    Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                                    Inches.of(2),
                                    KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                                    WHEEL_COF));

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    public Angle[] SwerveTurnAngle =
            new Angle[] {Radians.zero(), Radians.zero(), Radians.zero(), Radians.zero()};
    public Angle[] SwerveDriveAngle =
            new Angle[] {Radians.zero(), Radians.zero(), Radians.zero(), Radians.zero()};
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };

    public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    private final LoggedNetworkBoolean isTuningMode =
            new LoggedNetworkBoolean("/Tuning/Drive/tuningMode", false);

    private final Consumer<Pose2d> resetSimulationPoseCallBack;
    public ChassisSpeeds chassisSpeedsSetpoint = new ChassisSpeeds();

    private double lastSkidTimestamp = Double.NaN;

    private Field2d fieldPose = new Field2d();

    public Drive(
            GyroIO gyroIO, ModuleIO[] moduleIOS, Consumer<Pose2d> resetSimulationPoseCallBack) {
        this(
                gyroIO,
                moduleIOS[0],
                moduleIOS[1],
                moduleIOS[2],
                moduleIOS[3],
                resetSimulationPoseCallBack);
    }

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO,
            Consumer<Pose2d> resetSimulationPoseCallBack) {
        this.gyroIO = gyroIO;

        this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
        modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

        // Usage reporting for swerve template
        HAL.report(
                tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        configureAutoBuilder();

        Pathfinding.setPathfinder(new LocalADStar());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    fieldPose.getObject("path").setPoses(activePath);
                    Logger.recordOutput(
                            "Odometry/Trajectory",
                            activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });

        // Configure SysId
        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) ->
                                        Logger.recordOutput("Drive/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
        SmartDashboard.putData("Field", fieldPose);
    }

    private void configureAutoBuilder() {
        AutoBuilder.configure(
                () -> {
                    if (AutoCommandsKt.getUseOdometryOnlyInAuto()) {
                        return getOdometryPose();
                    }

                    return getPose();
                },
                this::resetOdometry,
                () -> chassisSpeeds,
                this::runVelocity,
                new PPHolonomicDriveController(
                        TunerConstants.autonomousTranslationPID,
                        TunerConstants.autonomousRotationPID),
                PP_CONFIG,
                () ->
                        DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
                this);
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        SwerveTurnAngle[0] = modules[0].getAngle().getMeasure();
        SwerveTurnAngle[1] = modules[1].getAngle().getMeasure();
        SwerveTurnAngle[2] = modules[2].getAngle().getMeasure();
        SwerveTurnAngle[3] = modules[3].getAngle().getMeasure();

        SwerveDriveAngle[0] = Radians.of(modules[0].getWheelRadiusCharacterizationPosition());
        SwerveDriveAngle[1] = Radians.of(modules[1].getWheelRadiusCharacterizationPosition());
        SwerveDriveAngle[2] = Radians.of(modules[2].getWheelRadiusCharacterizationPosition());
        SwerveDriveAngle[3] = Radians.of(modules[3].getWheelRadiusCharacterizationPosition());
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Log empty setpoint states when disabled
        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;

        chassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());
        Logger.recordOutput("SwerveChassisSpeeds/Measured", chassisSpeeds);

        for (int i = 0; i < sampleCount; i++) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] =
                        new SwerveModulePosition(
                                modulePositions[moduleIndex].distanceMeters
                                        - lastModulePositions[moduleIndex].distanceMeters,
                                modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Skid Detection
            double sampleTime = sampleTimestamps[i];
            double dt;
            if (Double.isNaN(lastSkidTimestamp)) {
                dt = 0.02;
            } else {
                dt = sampleTime - lastSkidTimestamp;
            }
            lastSkidTimestamp = sampleTime;

            boolean isSkidding = false;

            if (dt > 1e-4 && dt < 0.2) {
                Twist2d twistForSkid = kinematics.toTwist2d(moduleDeltas);
                isSkidding = isSkidding(twistForSkid, dt, moduleDeltas);
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Update gyro alert
            gyroDisconnectedAlert.set(
                    !gyroInputs.connected && ConstantsKt.getCURRENT_MODE() != Mode.SIM);

            Logger.recordOutput("Odometry/IsSkidding", isSkidding);
//            if (isSkidding) {
//                return;
//            }

            if (gyroInputs.connected) {
                // Update odometry
                BetterPoseEstimator.getInstance()
                        .addOdometryObservation(
                                new BetterPoseEstimator.OdometryObservation(
                                        Timer.getTimestamp(),
                                        getModulePositions(),
                                        gyroInputs.rollPosition,
                                        gyroInputs.pitchPosition,
                                        gyroInputs.yawPosition));
                BetterPoseEstimator.getInstance().setRobotVelocity(chassisSpeeds);
            }
        }
        fieldPose.setRobotPose(getPose());
    }

    private boolean isSkidding(
            Twist2d twistForSkid, double dt, SwerveModulePosition[] moduleDeltas) {
        double averagedOmega = twistForSkid.dtheta / dt;

        double minModulesLinearSpeed = Double.POSITIVE_INFINITY;
        double maxModulesLinearSpeed = 0.0;

        double totalDriveCurrent = 0.0;
        int usedModules = 0;

        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            double wheelSpeed = moduleDeltas[moduleIndex].distanceMeters / dt;

            Rotation2d azimuth = moduleDeltas[moduleIndex].angle;
            Translation2d vWheel = new Translation2d(wheelSpeed, azimuth);

            Translation2d r = getModuleTranslations()[moduleIndex];
            Translation2d vRot =
                    new Translation2d(-averagedOmega * r.getY(), averagedOmega * r.getX());

            Translation2d vLin = vWheel.minus(vRot);
            double moduleLinearSpeed = vLin.getNorm();

            if (moduleLinearSpeed > TunerConstants.modulesLinearSpeedEpsilonForSkid) {
                minModulesLinearSpeed = Math.min(minModulesLinearSpeed, moduleLinearSpeed);
                maxModulesLinearSpeed = Math.max(maxModulesLinearSpeed, moduleLinearSpeed);
                usedModules++;
            }

            totalDriveCurrent += modules[moduleIndex].getDriveMotorStatorCurrent();
        }

        boolean speedRatioSkid = false;
        if (usedModules >= 2 && minModulesLinearSpeed < Double.POSITIVE_INFINITY) {
            speedRatioSkid =
                    maxModulesLinearSpeed
                            > TunerConstants.modulesMinSpeedRatioForSkid * minModulesLinearSpeed;
        }

        return speedRatioSkid
                && totalDriveCurrent > TunerConstants.swerveMinTotalDriveCurrentForSkid;
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);

        chassisSpeedsSetpoint = speeds;
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules
     * will return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    public Command lock() {
        return runOnce(this::stopWithX);
    }

    public Command continousLock() {
        return run(this::stopWithX);
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.dynamic(direction));
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public void resetGyro(Angle resetHeading) {
        gyroIO.reset(resetHeading);
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    public AngularVelocity getGyroOmega() {
        return Units.RadiansPerSecond.of(gyroInputs.yawVelocityRadPerSec);
    }

    @AutoLogOutput(key = "SwerveChassisSpeeds/MeasuredFieldOriented")
    public ChassisSpeeds getFieldOrientedSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                kinematics.toChassisSpeeds(getModuleStates()), getRotation());
    }

    public LinearAcceleration getAccelerationX() {
        return gyroInputs.accelerationX;
    }

    public LinearAcceleration getAccelerationY() {
        return gyroInputs.accelerationY;
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return BetterPoseEstimator.getInstance().getEstimatedPose();
    }

    @AutoLogOutput(key = "Odometry/CompensatedRobot")
    public Pose2d getCompensatedPose() {
        ChassisSpeeds speeds = this.chassisSpeeds;
        return getPose()
                .plus(
                        new Transform2d(
                                new Translation2d(
                                        speeds.vxMetersPerSecond * compensationConstant,
                                        speeds.vyMetersPerSecond * compensationConstant),
                                new Rotation2d()));
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/OdometryPose")
    public Pose2d getOdometryPose() {
        return BetterPoseEstimator.getInstance().getOdometryPose();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public Rotation2d getGyroRotation() {
        return gyroInputs.yawPosition;
    }

    /** Resets the current odometry pose. USE BetterPoseEstimator instead! */
    public void resetOdometry(Pose2d pose) {
        resetSimulationPoseCallBack.accept(pose);
        BetterPoseEstimator.getInstance().resetPose(pose);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(
                    TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(
                    TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(
                    TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }

    @Override
    public void setVoltage(@NotNull Voltage voltage) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(voltage.in(Volts));
        }
    }

    @Override
    public @NotNull Function1<Voltage, Unit> getSetVoltageConsumer() {
        return (voltage) -> {
            setVoltage(voltage);
            return null;
        };
    }
}
