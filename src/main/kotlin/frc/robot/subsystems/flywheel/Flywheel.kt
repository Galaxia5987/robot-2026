package frc.robot.subsystems.flywheel

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.rps
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import org.team5987.annotation.command_enum.CommandEnum

object Flywheel : SubsystemBase(),PresetActions {
    private val motor =
        UniversalTalonFX(
            port = MAIN_MOTOR_PORT,
            config = MOTOR_CONFIG,
            subsystem = name,
            simGains = SIM_GAINS
        )

    private val velocityOut = VelocityVoltage(0.0)
    private val voltageOut = VoltageOut(0.0)
    @AutoLogOutput private var setpoint = 0.rps

    private val velocity
        get() = motor.inputs.velocity
    init {
        AUXILIARY_MOTORS_PORTS.forEach {
            UniversalTalonFX(port = it, config = MOTOR_CONFIG, subsystem = name)
                .setControl(
                    Follower(MAIN_MOTOR_PORT, MotorAlignmentValue.Aligned)
                        .withUpdateFreqHz(123.0)
                )
        }
    }

    private val calibrationVelocity =
        LoggedNetworkNumber(
            "/Tuning/Flywheel/calibrationFlywheelVelocity",
            40.0
        )

    fun setCalibrationVelocity(): Command = setVelocity {
        calibrationVelocity.get().rps
    }

    fun setVelocity(velocity: AngularVelocity): Command = setVelocity {
        setpoint = velocity
        velocity
    }

    fun setVelocity(velocity: () -> AngularVelocity): Command = run {
        setpoint = velocity()
        motor.setControl(velocityOut.withVelocity(setpoint))
    }

    fun stop() = setVelocity(0.rps)

    override fun setTarget(preset: Preset): Command = runOnce {
        setpoint = preset.velocity
        motor.setControl(velocityOut.withVelocity(setpoint))
    }

    fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageOut.withOutput(voltage))
    }

    override fun periodic() {
        motor.periodic()
    }
}
