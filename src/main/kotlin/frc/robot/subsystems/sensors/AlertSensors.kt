package frc.robot.subsystems.sensors

import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.lib.logged_output.LoggedOutputManager.runOnce

class AlertSensors {
    private val disconnectedAlertSpindexer =
        Alert("Spindexer's Sensor's motor is disconnected", AlertType.kWarning)

    private val disconnectedAlertTop =
        Alert("Top Sensor's motor is disconnected", AlertType.kWarning)

    private val disconnectedAlertAuxTop =
        Alert("AuxTop's Sensor's motor is disconnected", AlertType.kWarning)

    fun isConnected(condition: Boolean) : Command = runOnce {
        if (condition)
            disconnectedAlertSpindexer.set(false)
        else
            disconnectedAlertSpindexer.set(true)
    }

    fun isConnectedTop(condition: Boolean) : Command = runOnce {
        if (condition)
            disconnectedAlertTop.set(false)
        else
            disconnectedAlertTop.set(true)
    }
    fun isConnectedAuxTop(condition: Boolean) : Command = runOnce {
        if (condition)
            disconnectedAlertAuxTop.set(false)

        else
            disconnectedAlertAuxTop.set(true)
    }
}