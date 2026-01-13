package frc.robot.lib.universal_motor

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.sec
import frc.robot.lib.extensions.volts
import org.team5987.annotation.Logged

/**
 * Interface that defines the common motor input/output operations for both real
 * and simulated motors.
 */
interface MotorIO {
    /** Current configuration for the TalonFX motor. */
    val config: TalonFXConfiguration

    /** Container for the current state of motor inputs. */
    val inputs: LoggedMotorInputs

    /**
     * Sends a control request to the motor (e.g., velocity, position, voltage
     * control).
     *
     * @param controlRequest The control request to apply.
     */
    fun setRequest(controlRequest: ControlRequest) {}

    fun resetInternalEncoder(angle: Angle = 0.deg) {}

    fun applyConfiguration(configuration: TalonFXConfiguration) {}

    /**
     * Updates the latest inputs from the motor or simulation. Should be called
     * periodically.
     */
    fun updateInputs() {}

    /**
     * Base class representing the observable inputs from a motor. This class
     * generates the logging class `LoggedMotorInputs`
     */
    @Logged
    open class MotorInputs {
        /** Current position of the motor shaft. */
        var position: Angle = 0.deg

        /**
         * Linear distance the motor has moved, accounting for gear ratio and
         * wheel diameter. If not a linear system will not change
         */
        var distance: Distance = 0.m

        var velocity: AngularVelocity = 0.deg.per(sec)

        /** Voltage applied to the motor. */
        var voltage: Voltage = 0.volts

        /** Current drawn by the motor. */
        var current: Current = 0.amps
        var absoluteEncoderPositionNoOffset: Angle = 0.deg
    }
}
