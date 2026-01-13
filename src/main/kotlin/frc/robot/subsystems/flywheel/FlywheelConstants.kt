package frc.robot.subsystems.flywheel

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.rps

val GAINS = Gains(kP = 0.0, kS = 0.0, kV = 0.0)
val SIM_GAINS = Gains(kP = 0.0, kS = 0.0)

val MOTOR_PORT = 0

enum class AUXILIARY_MOTORS_PORTS(val port: Int) {
    MOTOR_PORT2(0),
    MOTOR_PORT3(1)
}

enum class Preset(val velocity: AngularVelocity) {
    NEAR(30.0.rps),
    MEDIUM(40.0.rps),
    FAR(50.0.rps)
}

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.CounterClockwise_Positive
            }
        Feedback = FeedbackConfigs().apply { RotorToSensorRatio = 0.0 }
        Slot0 = GAINS.toSlotConfig()

        CurrentLimits =
            createCurrentLimits(supplyCurrentPeakDifference = 10.amps)
    }
