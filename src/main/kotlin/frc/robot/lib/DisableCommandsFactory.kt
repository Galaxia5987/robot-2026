package frc.robot.lib

import com.ctre.phoenix6.controls.CoastOut
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.universal_motor.UniversalTalonFX

private val coastOut = CoastOut()

fun createDisableTriggerForCoast(motor: UniversalTalonFX) {
    val coast = { motor.setControl(coastOut) }
    Trigger { DriverStation.isDisabled() }
        .onTrue(Commands.runOnce(coast).ignoringDisable(true))
    coast.invoke()
}
