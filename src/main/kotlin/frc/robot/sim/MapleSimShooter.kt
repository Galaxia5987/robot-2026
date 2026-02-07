package frc.robot.sim

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.calculateVelocity
import frc.robot.drive
import frc.robot.field.HUB_LOCATION
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.mps
import frc.robot.lib.extensions.toRotation2d
import frc.robot.lib.extensions.toTranslation3d
import frc.robot.states.setpoints_manager.shooting_modes.distanceFromGoal
import frc.robot.states.shooting.ShootingState
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.turret.Turret
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly

val HUB_HEIGHT = 177.cm // ONLY USED FOR MAPLE SIM!!!!

class MapleSimShooter(private val mapleSimIntake: MapleSimIntake) {
    fun createFuelOnFly(): RebuiltFuelOnFly {
        val robotSpeeds = drive.chassisSpeeds

        val fuelOnFly =
            RebuiltFuelOnFly(
                drive.pose.translation,
                Translation2d((-116).mm, 220.5.mm)
                    .rotateBy(drive.pose.rotation),
                robotSpeeds,
                Turret.inputs.position.toRotation2d() + drive.pose.rotation,
                0.47865.m,
                if (distanceFromGoal > 10.m) 13.mps
                else
                    calculateVelocity(
                            distanceFromGoal[m],
                            robotSpeeds.vxMetersPerSecond,
                            robotSpeeds.vyMetersPerSecond
                        )
                        .mps + 1.mps,
                90.deg - Hood.inputs.position - 15.deg
            )
        fuelOnFly
            .withTargetPosition { HUB_LOCATION.toTranslation3d(HUB_HEIGHT) }
            .withTargetTolerance(
                Translation3d(122.186335.cm, 122.186335.cm, 3.cm)
            )
        return fuelOnFly
    }

    fun createFuelCommand(): Command =
        (Commands.runOnce({
                    mapleSimIntake.obtainBallFromIntake()
                    SimulatedArena.getInstance()
                        .addGamePieceProjectile(createFuelOnFly())
                })
                .andThen(Commands.waitSeconds(0.1)))
            .repeatedly()

    private val launchBallTrigger =
        ShootingState.SHOOTING.trigger.whileTrue(createFuelCommand())
}
