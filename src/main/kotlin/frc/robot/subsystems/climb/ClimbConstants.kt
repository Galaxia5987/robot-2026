package frc.robot.subsystems.climb

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.Gains
import frc.robot.lib.LoggedNetworkGains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.deg
import org.team5987.annotation.command_enum.CommandEnum

const val MAIN_PORT = 0

val REAL_GAINS =
    LoggedNetworkGains(
        "ClimbRealGains",
        1.0,
        kG = 10.0,
        gravityTypeValue = GravityTypeValue.Arm_Cosine
    ) // TODO calibrations
val SIM_GAINS = Gains(1.0, kG = 10.0)

const val GEAR_RATION = 1.0

val TOLERANCE = 0.5.deg

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        Slot0 = REAL_GAINS.toSlotConfig()
        MotionMagic = REAL_GAINS.toMotionMagicConfig()
        CurrentLimits = createCurrentLimits()
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.Clockwise_Positive
            }
    }

@CommandEnum
enum class ClimbLevels(val angle: Angle) {
    RETRACTED(0.deg),
    EXTENDED(90.deg),
    ENGAGED(180.deg),
}
