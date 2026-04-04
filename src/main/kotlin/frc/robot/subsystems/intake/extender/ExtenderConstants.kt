package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.*
import org.team5987.annotation.command_enum.CommandEnum

val DIAMETER = 25.4.mm

val EXTENDER_SETPOINT_TOLERANCE = 4.cm

const val PORT = 11

const val GEAR_RATIO = 1 / 3.17

val SIM_GAINS = Gains(kP = 1.4, kD = 0.3)

val REAL_GAINS = Gains(kP = 3.5, kI = 2.0, kS = 2.0, kV = 2.5)

// Minimum current for when the motor is stalled (can't close anymore)
val STATOR_STALL_CURRENT = 50.amps

val STALL_DEBOUNCE = 0.1.sec

val FORWARD_LIMIT = 12.15.rot

val CLOSE_VOLTAGE = (-6).volts

val PUMPING_VOLTAGE = (-3).volts

val RESET_VOLTAGE = 6.0.volts

val CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.CounterClockwise_Positive
            }

        Slot0 = REAL_GAINS.toSlotConfig()
        SoftwareLimitSwitch =
            SoftwareLimitSwitchConfigs().apply {
                ForwardSoftLimitEnable = true
                ReverseSoftLimitEnable = true
                ForwardSoftLimitThreshold = FORWARD_LIMIT[rot]
                ReverseSoftLimitThreshold = -0.5
            }

        CurrentLimits = createCurrentLimits(25.amps, 5.amps)
    }

@CommandEnum
enum class ExtenderPositions(val distance: Distance) {
    OPEN(0.304.meters),
    CLOSE(0.0.meters)
}
