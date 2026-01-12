package frc.robot.lib

import edu.wpi.first.wpilibj2.command.button.Trigger
import java.util.function.BooleanSupplier
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean

class LoggedNetworkTrigger(key: String, booleanSupplier: BooleanSupplier) :
    Trigger({ false }) {

    private val loggedNetworkBoolean =
        LoggedNetworkBoolean(key, booleanSupplier.asBoolean)

    init {
        this.or { loggedNetworkBoolean.get() }
    }
}
