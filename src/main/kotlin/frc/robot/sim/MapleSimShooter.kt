package frc.robot.sim

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
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
import frc.robot.lib.getTranslation2d
import frc.robot.states.setpoints_manager.shooting_modes.distanceFromGoal
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.turret.Turret
import frc.robot.subsystems.shooter.turret.turretAngleToHub
import java.util.function.Supplier
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly

val HUB_HEIGHT = 177.cm // ONLY USED FOR MAPLE SIM!!!!

object MapleSimShooter {
    fun createFuelOnFly(): RebuiltFuelOnFly {
        val robotSpeeds = drive.chassisSpeeds
        val fuelOnFly = RebuiltFuelOnFly(
            drive.pose.translation,
            Translation2d((-116).mm, 220.5.mm).rotateBy(drive.pose.rotation),
            robotSpeeds,
            Turret.inputs.position.toRotation2d() + drive.pose.rotation,
            0.47865.m,
            calculateVelocity(distanceFromGoal[m], robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond).mps + 1.mps,
            90.deg - Hood.inputs.position - 15.deg
        )
        fuelOnFly
            .withTargetPosition { HUB_LOCATION.toTranslation3d(HUB_HEIGHT) }
            .withTargetTolerance(Translation3d(122.186335.cm, 122.186335.cm, 3.cm))
        return fuelOnFly
    }
}
