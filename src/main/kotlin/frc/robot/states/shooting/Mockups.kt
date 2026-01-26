package frc.robot.states.shooting

import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.rps

fun <T : SubsystemBase> T.aimingSetpoint(): AngularVelocity = 0.0.rps
