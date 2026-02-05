package frc.robot.sim

import edu.wpi.first.math.geometry.Translation3d
import frc.robot.calculateVelocity
import frc.robot.drive
import frc.robot.field.HUB_HEIGHT
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

object MapleSimShooter {
    fun createFuelOnFly(): RebuiltFuelOnFly {
        val robotSpeeds = drive.chassisSpeeds
        val fuelOnFly = RebuiltFuelOnFly(
            drive.pose.translation,
            SHOOTER_TO_ROBOT_POSE,
            robotSpeeds,
            turretAngleToHub.toRotation2d(),
            0.47865.m,
            calculateVelocity(distanceFromGoal[m], robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond).mps,
            90.deg - Hood.inputs.position - 15.deg
        )
        fuelOnFly
            .withTargetPosition { HUB_LOCATION.toTranslation3d(HUB_HEIGHT) }
            .withTargetTolerance(Translation3d(10.0.cm, 10.cm, 10.cm))
            .withHitTargetCallBack { print("HIT") }
        return fuelOnFly
    }
}
