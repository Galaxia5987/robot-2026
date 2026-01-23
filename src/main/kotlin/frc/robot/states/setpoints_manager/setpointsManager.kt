package frc.robot.states.setpoints_manager

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.field.DEPOT_LOCATION
import frc.robot.field.HUB_LOCATION
import frc.robot.field.OUTPOST_LOCATION
import frc.robot.field.inAllianceZone
import frc.robot.field.outOfAllianceZone_AboveCrossLine
import frc.robot.field.outOfAllianceZone_BelowCrossLine

private fun <T: SubsystemBase> getSetpointByOverrides (pose : Translation2d, subsystem: T) {

}

fun <T: SubsystemBase> getAimingToSetpoint(subsystem: T) {
    var target = Translation2d()
    if(inAllianceZone.asBoolean) {
        target = HUB_LOCATION
    }else if(outOfAllianceZone_AboveCrossLine.asBoolean){
        target = DEPOT_LOCATION
    }else if(outOfAllianceZone_BelowCrossLine.asBoolean){
        target = OUTPOST_LOCATION
    }
}