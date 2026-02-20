package frc.robot.subsystems.sensors

import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.lib.logged_output.LoggedOutputManager.runOnce

class AlertSensors {
    private val disconnectedAlertSpindexer =
        Alert("Spindexer's Sensor is disconnected", AlertType.kWarning)

    private val connectedAlertSpindexer =
        Alert("Spindexer's Sensor is connected", AlertType.kInfo)

    private val disconnectedAlertTop =
        Alert("Top Sensor is disconnected", AlertType.kWarning)

    private val connectedAlertTop =
        Alert("Top Sensor is connected", AlertType.kInfo)

    private val disconnectedAlertAuxTop =
        Alert("AuxTop's Sensor is disconnected", AlertType.kWarning)

    private val connectedAlertAuxTop =
        Alert("AuxTop's Sensor is connected", AlertType.kInfo)

    fun isConnectedSpindexer(condition: Boolean) : Command = runOnce {
        disconnectedAlertSpindexer.set(!condition)
        connectedAlertSpindexer.set(condition)

    }

    fun isConnectedTop(condition: Boolean) : Command = runOnce {
        disconnectedAlertTop.set(!condition)
        connectedAlertTop.set(condition)

    }
    fun isConnectedAuxTop(condition: Boolean) : Command = runOnce {
        disconnectedAlertAuxTop.set(!condition)
        connectedAlertAuxTop.set(condition)
    }
}