package frc.robot.states.setpoints_manager

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.field.*
import frc.robot.isAuto
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.not
import frc.robot.lib.extensions.onFalse
import frc.robot.lib.extensions.onTrue
import frc.robot.lib.extensions.rotationToPoint
import frc.robot.lib.extensions.toPose
import frc.robot.states.DriverOverrides
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.turret.TURRET_TO_ROBOT
import frc.robot.subsystems.shooter.turret.constraintTurretLimits
import org.littletonrobotics.junction.Logger

object SetpointsManager : SubsystemBase() {
    private val goalHubTrigger =
        inExtendedAllianceZone
            .or(isAuto)
            .onTrue(
                runOnce({
                        currentGoal = HUB_TRANSLATION.toPose()
                        isFeeding = false
                    })
                    .ignoringDisable(true)
            )

    private val goalDepotTrigger =
        isCloserToDepotSide
            .and(!inExtendedAllianceZone)
            .and(inClimbRectangle.negate())
            .and(isAuto.negate())
            .onTrue(
                runOnce({
                        currentGoal = DEPOT_TRANSLATION.toPose()
                        isFeeding = true
                    })
                    .ignoringDisable(true)
            )

    private val goalOutpostTrigger =
        isCloserToDepotSide
            .negate()
            .and(!inExtendedAllianceZone)
            .and(inClimbRectangle.negate())
            .and(isAuto.negate())
            .onTrue(
                runOnce({
                        currentGoal = OUTPOST_LOCATION.toPose()
                        isFeeding = true
                    })
                    .ignoringDisable(true)
            )

    val isShootingOnMove = Trigger {
        shootingType == ShootingType.SHOOT_ON_MOVE
    }

    val isUsingInterpolation = Trigger {
        shootingType == ShootingType.INTERPOLATION
    }

    val isUsingStaticSetpoints = Trigger { shootingType == ShootingType.STATIC }

    val isUsingFeeding =
        Trigger { isFeeding }
            .onTrue(
                Flywheel.setLowCurrentLimits(),
                PreShooter.setLowCurrentLimits(),
            )
            .onFalse(
                Flywheel.setRegularCurrentLimits(),
                PreShooter.setLowCurrentLimits(),
            )

    var currentGoal: Pose2d = HUB_TRANSLATION.toPose()
    var isFeeding = false
    val shootingType
        get() =
            when {
                DriverOverrides.StaticShootingOverride.trigger.asBoolean ->
                    ShootingType.STATIC
                DriverOverrides.ShootingCalibrationOverride.trigger.asBoolean ->
                    ShootingType.CALIBRATION
                isFeeding -> ShootingType.FEEDING
                DriverOverrides.ShootOnMoveOverride.trigger.asBoolean ->
                    ShootingType.SHOOT_ON_MOVE
                else -> ShootingType.INTERPOLATION
            }

    var turretTranslationFieldOriented: Translation2d = Translation2d()
        private set

    var compensatedTurretTranslationFieldOriented: Translation2d =
        Translation2d()
        private set

    private var turretAngleFromRobotToHub: Rotation2d = Rotation2d()

    var turretAngleToHub: Angle = Degrees.zero()
        private set

    var constrainedTurretAngleToHub: Angle = Degrees.zero()
        private set

    var turretDistanceFromGoal: Distance = Meters.zero()
        private set

    var compensatedTurretDistanceFromGoal: Distance = Meters.zero()
        private set

    override fun periodic() {
        val rotatedTurretOffset = TURRET_TO_ROBOT.rotateBy(drive.pose.rotation)

        turretTranslationFieldOriented =
            drive.pose.translation.plus(rotatedTurretOffset)

        compensatedTurretTranslationFieldOriented =
            drive.compensatedPose.translation.plus(rotatedTurretOffset)

        turretAngleFromRobotToHub =
            turretTranslationFieldOriented.rotationToPoint(
                currentGoal.translation
            )

        turretAngleToHub =
            (drive.pose.rotation - turretAngleFromRobotToHub).measure

        constrainedTurretAngleToHub = constraintTurretLimits(turretAngleToHub)

        turretDistanceFromGoal =
            turretTranslationFieldOriented
                .getDistance(currentGoal.translation)
                .m
        compensatedTurretDistanceFromGoal =
            compensatedTurretTranslationFieldOriented
                .getDistance(currentGoal.translation)
                .m

        Logger.recordOutput("StateMachines/Shooting/currentGoal", currentGoal)
        Logger.recordOutput(
            "StateMachines/Shooting/turretDistanceFromGoal",
            turretDistanceFromGoal
        )
        Logger.recordOutput("StateMachines/Shooting/shootingType", shootingType)
    }
}
