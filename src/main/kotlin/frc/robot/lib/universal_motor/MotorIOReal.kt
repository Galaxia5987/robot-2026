package frc.robot.lib.universal_motor

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.extensions.toDistance

/**
 * Real implementation of [MotorIO] for interacting with actual TalonFX
 * hardware.
 *
 * @param port The CAN ID of the motor controller.
 * @param canBus The CAN bus name (use empty string for default).
 * @param config The TalonFX configuration to apply on startup.
 * @param gearRatio The gear ratio between motor rotations and mechanism output.
 * @param diameter The diameter of the wheel/spool if used in a linear system.
 */
class MotorIOReal(
    private val port: Int,
    private val canBus: CANBus,
    override val config: TalonFXConfiguration,
    private val gearRatio: Double,
    private val diameter: Distance,
    private val absoluteEncoderOffset: Angle,
) : MotorIO {
    override val inputs = LoggedMotorInputs()
    private val motor = TalonFX(port, canBus)

    init {
        motor.configurator.apply(config)
    }

    override fun setRequest(controlRequest: ControlRequest) {
        motor.setControl(controlRequest)
    }

    override fun resetInternalEncoder(angle: Angle) {
        motor.setPosition(angle)
    }

    override fun applyConfiguration(configuration: TalonFXConfiguration) {
        motor.configurator.apply(configuration)
    }

    override fun updateInputs() {
        inputs.current = motor.supplyCurrent.value
        inputs.position = motor.position.value
        inputs.voltage = motor.motorVoltage.value
        inputs.velocity = motor.velocity.value
        inputs.distance = motor.position.value.toDistance(diameter, gearRatio)
        inputs.absoluteEncoderPositionNoOffset =
            motor.position.value - absoluteEncoderOffset
    }
}
