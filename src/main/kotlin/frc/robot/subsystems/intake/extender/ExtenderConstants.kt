package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.*
import org.team5987.annotation.command_enum.CommandEnum

val DIAMETER = 25.4.mm

val TOLERANCE = 4.cm

const val PORT = 11

const val GEAR_RATIO = 1 / 3.17
// TODO: actual value

val SIM_GAINS = Gains(kP = 1.4, kD = 0.3)

val REAL_GAINS = Gains(kP = 3.5, kI = 2.0, kS = 2.0, kV = 2.5)

val FORWARD_LIMIT = 11.63.rot
val REVERSE_LIMIT = 0.rot

val CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.CounterClockwise_Positive
            }

        Slot0 = REAL_GAINS.toSlotConfig()
        SoftwareLimitSwitch = SoftwareLimitSwitchConfigs().apply {
            ForwardSoftLimitEnable = true
            ReverseSoftLimitEnable = true
            ForwardSoftLimitThreshold = FORWARD_LIMIT[rot]
            ReverseSoftLimitThreshold = REVERSE_LIMIT[rot]
        }

        CurrentLimits = createCurrentLimits(30.amps, 5.amps)
    }

@CommandEnum
enum class ExtenderPositions(val distance: Distance) {
    OPEN(0.303224.meters),
    CLOSE(0.004.meters)
}
