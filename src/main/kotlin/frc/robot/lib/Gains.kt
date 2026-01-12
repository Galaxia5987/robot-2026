package frc.robot.lib

import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rad_ps
import frc.robot.lib.extensions.rad_ps_ps
import frc.robot.lib.extensions.rps
import frc.robot.lib.extensions.rps_squared
import kotlin.apply
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

data class Gains(
    var kP: Double = 0.0,
    var kI: Double = 0.0,
    var kD: Double = 0.0,
    var kS: Double = 0.0,
    var kV: Double = 0.0,
    var kA: Double = 0.0,
    var kG: Double = 0.0,
    val motionMagicGains: MotionMagicGains = MotionMagicGains()
) {
    /**
     * A function to convert a [Gains] type to the [Slot0Configs] that the motor
     * uses.
     */
    fun toSlotConfig() =
        Slot0Configs().apply {
            kP = this@Gains.kP
            kI = this@Gains.kI
            kD = this@Gains.kD
            kA = this@Gains.kA
            kS = this@Gains.kS
            kV = this@Gains.kV
            kG = this@Gains.kG
        }
}

data class MotionMagicGains(
    var cruiseVelocity: AngularVelocity = 0.rps,
    var acceleration: AngularAcceleration = 0.rps_squared,
    var jerk: Double = 0.0 // m/s^3
) {
    fun toMotionMagicConfig() =
        MotionMagicConfigs().apply {
            MotionMagicCruiseVelocity =
                this@MotionMagicGains.cruiseVelocity[rps]
            MotionMagicAcceleration =
                this@MotionMagicGains.acceleration[rps_squared]
            MotionMagicJerk = this@MotionMagicGains.jerk
        }
}

class LoggedNetworkGains(
    name: String,
    kP: Double = 0.0,
    kI: Double = 0.0,
    kD: Double = 0.0,
    kS: Double = 0.0,
    kV: Double = 0.0,
    kA: Double = 0.0,
    kG: Double = 0.0,
    cruiseVelocity: AngularVelocity = 0.rps,
    acceleration: AngularAcceleration = 0.rps_squared,
    jerk: Double = 0.0, // m/s
    key: String =
        (Throwable().stackTrace[1]?.fileName?.substringBeforeLast('.') + ""),
) {
    private val path = "/Tuning/$key/$name"
    val kP: LoggedNetworkNumber = LoggedNetworkNumber("$path/kP", kP)
    val kI: LoggedNetworkNumber = LoggedNetworkNumber("$path/kI", kI)
    val kD: LoggedNetworkNumber = LoggedNetworkNumber("$path/kD", kD)
    val kS: LoggedNetworkNumber = LoggedNetworkNumber("$path/kS", kS)
    val kV: LoggedNetworkNumber = LoggedNetworkNumber("$path/kV", kV)
    val kA: LoggedNetworkNumber = LoggedNetworkNumber("$path/kA", kA)
    val kG: LoggedNetworkNumber = LoggedNetworkNumber("$path/kG", kG)
    var jerk: LoggedNetworkNumber? = null
    var cruiseVelocity: LoggedNetworkNumber? = null
    var acceleration: LoggedNetworkNumber? = null

    init {
        if (
            jerk != 0.0 ||
                cruiseVelocity[rad_ps] != 0.0 ||
                acceleration[rad_ps_ps] != 0.0
        ) {
            this@LoggedNetworkGains.jerk =
                LoggedNetworkNumber("$path/jerk", jerk)
            this@LoggedNetworkGains.cruiseVelocity =
                LoggedNetworkNumber(
                    "$path/cruiseVelocity",
                    cruiseVelocity[rad_ps]
                )
            this@LoggedNetworkGains.acceleration =
                LoggedNetworkNumber(
                    "$path/acceleration",
                    acceleration[rad_ps_ps]
                )
        }
    }

    fun toSlotConfig() =
        Slot0Configs().apply {
            kP = this@LoggedNetworkGains.kP.get()
            kI = this@LoggedNetworkGains.kI.get()
            kD = this@LoggedNetworkGains.kD.get()
            kA = this@LoggedNetworkGains.kA.get()
            kS = this@LoggedNetworkGains.kS.get()
            kV = this@LoggedNetworkGains.kV.get()
            kG = this@LoggedNetworkGains.kG.get()
        }

    fun toMotionMagicConfig() =
        MotionMagicConfigs().apply {
            MotionMagicCruiseVelocity =
                this@LoggedNetworkGains.cruiseVelocity?.get() ?: 0.0
            MotionMagicAcceleration =
                this@LoggedNetworkGains.acceleration?.get() ?: 0.0
            MotionMagicJerk = this@LoggedNetworkGains.jerk?.get() ?: 0.0
        }
}
