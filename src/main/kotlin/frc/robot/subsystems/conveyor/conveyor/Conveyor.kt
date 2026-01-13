package frc.robot.subsystems.conveyor.conveyor

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.sysid.SysIdable
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

object Conveyor : SubsystemBase(), ConveyorVelocityCommandFactory, SysIdable {

    private val mainMotor =
        UniversalTalonFX(
            MAIN_MOTOR_ID,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATIO,
            simGains = SIM_GAINS
        )

    private val auxMotor =
        UniversalTalonFX(
                AUX_MOTOR_ID,
                config = MOTOR_CONFIG,
                gearRatio = GEAR_RATIO,
                simGains = SIM_GAINS
            )
            .apply {
                setControl(Follower(MAIN_MOTOR_ID, MotorAlignmentValue.Aligned))
            } // TODO is using double motors ?

    private val velocityVoltage = VelocityVoltage(0.0)
    private val voltageRequest = VoltageOut(0.0)

    @LoggedOutput(LogLevel.DEV) var setpoint = ConveyorVelocity.STOP

    @LoggedOutput(LogLevel.DEV)
    val isAtSetpoint = Trigger {
        mainMotor.inputs.velocity.isNear(setpoint.velocity, SETPOINT_TOLERANCE)
    }

    override fun setTarget(value: ConveyorVelocity): Command = runOnce {
        mainMotor.setControl(velocityVoltage.withVelocity(value.velocity))
        setpoint = value
    }

    override fun setVoltage(voltage: Voltage) {
        mainMotor.setControl(voltageRequest.withOutput(voltage))
    }

    override fun periodic() {
        mainMotor.periodic()
    }
}
