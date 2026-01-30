package frc.robot

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.extensions.*
import frc.robot.lib.getPose3d
import frc.robot.lib.getRotation3d
import frc.robot.lib.getTranslation3d
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.roller.Roller
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.turret.Turret
import frc.robot.subsystems.spindexer.Spindexer
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import kotlin.math.cos
import kotlin.math.sin

private val swerveModulePose: Array<Translation2d> =
    Drive.getModuleTranslations()

private val kWheelRadius = 0.0508.m

private fun getSwerveModulePoseTurn(
    moduleX: Double,
    moduleY: Double,
    moduleYaw: Angle
): Pose3d {
    return Pose3d(
        Translation3d(moduleX, moduleY, kWheelRadius[m]),
        getRotation3d(yaw = moduleYaw)
    )
}

private fun getSwerveModulePoseDrive(
    moduleX: Double,
    moduleY: Double,
    moduleYaw: Angle,
    modulePitch: Angle
): Pose3d {

    return Pose3d(
        Translation3d(moduleX, moduleY, kWheelRadius[m]),
        getRotation3d(yaw = moduleYaw, pitch = modulePitch)
    )
}

private fun getAllSwerveModulePoseTurn(): Array<Pose3d> {
    val swervePosesTurn: Array<Pose3d> =
        arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d())
    for (i in 0..3) {
        swervePosesTurn[i] =
            getSwerveModulePoseTurn(
                swerveModulePose[i].x,
                swerveModulePose[i].y,
                drive.SwerveTurnAngle[i]
            )
    }
    return swervePosesTurn
}

private fun getAllSwerveModulePoseDrive(): Array<Pose3d> {
    val swervePosesDrive: Array<Pose3d> =
        arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d())

    for (i in 0..3) {
        swervePosesDrive[i] =
            getSwerveModulePoseDrive(
                swerveModulePose[i].x,
                swerveModulePose[i].y,
                drive.SwerveTurnAngle[i],
                drive.SwerveDriveAngle[i]
            )
    }
    return swervePosesDrive
}

private object Swerve {

}

private object Intake {
    private val INTAKE_ANGLE = 9.deg
    private val EXTENDER_ANGLE_POSE: Translation3d
        get() = getTranslation3d(
            x = Extender.inputs.distance * cos(INTAKE_ANGLE[rad]),
            z = -Extender.inputs.distance * sin(
                INTAKE_ANGLE[rad]
            )
        )

    val extender: Pose3d
        get() = getPose3d(
            getTranslation3d(80.mm, 10.mm, 248.mm) + EXTENDER_ANGLE_POSE
        )

    val extendingHopper: Pose3d
        get() = getPose3d(
            getTranslation3d(346.mm, 10.mm, 235.mm) + Extender.inputs.distance.toX()
        )

    val roller1
        get() = getPose3d(
            getTranslation3d(323.mm, 10.mm, 197.mm) + EXTENDER_ANGLE_POSE,
            Roller.inputs.position.toPitch()
        )

    val roller2
        get() = getPose3d(
            getTranslation3d(260.mm, 10.mm, 197.mm) + EXTENDER_ANGLE_POSE,
            Roller.inputs.position.toPitch()
        )
}

private object Shooter {
    private val turretRotation
        get() = Turret.inputs.position.toYaw()

    private val turretTranslation
        get() = getTranslation3d((-116).mm, 220.5.mm, 355.mm)

    private val hoodTranslation
        get() = getTranslation3d((-48).mm, 220.5.mm, 435.mm).rotateAround(turretTranslation, turretRotation)

    private val hoodRotation
        get() = Hood.inputs.position.toPitch() + turretRotation

    val turret
        get() = getPose3d(
            turretTranslation, turretRotation
        )

    val hood
        get() = getPose3d(
            hoodTranslation,
            hoodRotation
        )

    val shooterMainRoller
        get() = getPose3d(
            getTranslation3d((-48).mm, 220.5.mm, 436.mm).rotateAround(turretTranslation, turretRotation),
            turretRotation
        ).plus(Transform3d(Translation3d(), Flywheel.inputs.position.toPitch()))

    val hoodRoller: Pose3d
        get() = hood.plus(
            Transform3d(
                getTranslation3d((-247).mm, 220.5.mm, 489.mm) - getTranslation3d(
                    (-48).mm,
                    220.5.mm,
                    435.mm
                ), Flywheel.inputs.position.toPitch()
            )
        )
}

private object Conveyors {
    val spindexer
        get() = getPose3d(
            getTranslation3d((-26).mm, 43.mm, 36.66200.mm),
            Spindexer.inputs.position.toYaw()
        )
    val firstRoller
        get() = getPose3d(
            getTranslation3d((-63).mm, 90.mm, 211.mm),
            PreShooter.inputs.position.toPitch()
        )
    val secondRoller
        get() = getPose3d(
            getTranslation3d((-22).mm, 130.mm, 295.mm),
            PreShooter.inputs.position.toPitch()
        )

    val thirdRoller
        get() = getPose3d(
            getTranslation3d((-23.5).mm, 315.mm, 295.mm),
            PreShooter.inputs.position.toPitch()
        )
}

private object Climb {
    val grabber
        get() = getPose3d(
            getTranslation3d((-30).mm, (-242).mm, 420.mm)
        )
    val wrist
        get() = getPose3d(getTranslation3d((-250).mm, (-380).mm, 460.mm))
}

private val subsystemPoseArray = Array(14) { Pose3d() }

@LoggedOutput(key = "Visualization/mechanismPoses", level = LogLevel.COMP)
fun mechanismPoses(): Array<Pose3d> {
    //    val swerveModulesPoses = getAllSwerveModulePoseDrive()
    //
    //    swerveModulesPoses.forEachIndexed { i, modulePose ->
    //        subsystemPoseArray[2 * i + 1] = modulePose
    //    }

    subsystemPoseArray[0] = Intake.extender
    subsystemPoseArray[1] = Intake.extendingHopper
    subsystemPoseArray[2] = Intake.roller1
    subsystemPoseArray[3] = Intake.roller2
    subsystemPoseArray[4] = Shooter.turret
    subsystemPoseArray[5] = Shooter.hood
    subsystemPoseArray[6] = Shooter.shooterMainRoller
    subsystemPoseArray[7] = Shooter.hoodRoller
    subsystemPoseArray[8] = Conveyors.spindexer
    subsystemPoseArray[9] = Conveyors.firstRoller
    subsystemPoseArray[10] = Conveyors.secondRoller
    subsystemPoseArray[11] = Conveyors.thirdRoller
    subsystemPoseArray[12] = Climb.grabber
    subsystemPoseArray[13] = Climb.wrist

    return subsystemPoseArray
}

// For Tuning !!
private val tuningSubsystemPoses =
    arrayOf(
        TunablePose3d(
            "/Tuning/Visualization/IntakeExtender",
            Intake.extender
        ),
        TunablePose3d(
            "/Tuning/Visualization/ExtendingHopper",
            Intake.extendingHopper
        ),
        TunablePose3d(
            "/Tuning/Visualization/IntakeRoller1",
            Intake.roller1
        ),
        TunablePose3d(
            "/Tuning/Visualization/IntakeRoller2",
            Intake.roller2
        ),
        TunablePose3d("/Tuning/Visualization/Turret", Shooter.turret),
        TunablePose3d("/Tuning/Visualization/Hood", Shooter.hood),
        TunablePose3d(
            "/Tuning/Visualization/ShooterMainRoller",
            Shooter.shooterMainRoller
        ),
        TunablePose3d(
            "/Tuning/Visualization/HoodRoller",
            Shooter.hoodRoller
        ),
        TunablePose3d("/Tuning/Visualization/Spindexer", Conveyors.spindexer),
        TunablePose3d(
            "/Tuning/Visualization/PreShooterRoller",
            Conveyors.firstRoller
        ),
        TunablePose3d(
            "/Tuning/Visualization/PreShooterSecondRoller",
            Conveyors.secondRoller
        ),
        TunablePose3d(
            "/Tuning/Visualization/PreShooterThirdRoller",
            Conveyors.thirdRoller
        ),
        TunablePose3d(
            "/Tuning/Visualization/ClimbGrabber",
            Climb.grabber
        ),
        TunablePose3d(
            "/Tuning/Visualization/ClimbWrist",
            Climb.wrist
        ),
    )

@LoggedOutput(key = "Visualization/TuningMechanismPoses", level = LogLevel.COMP)
fun getTuningSubsystemPoses(): Array<Pose3d> =
    tuningSubsystemPoses.map { it.get() }.toTypedArray()
