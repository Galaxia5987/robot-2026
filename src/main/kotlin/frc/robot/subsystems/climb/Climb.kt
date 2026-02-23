package frc.robot.subsystems.climb

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.toAngle
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

object Climb : SubsystemBase(), ClimbLevelsCommandFactory {
    private val motor =
        UniversalTalonFX(
            MAIN_PORT,
            config = MOTOR_CONFIG,
            simGains = SIM_GAINS,
            gearRatio = GEAR_RATIO,
            logConfig =
                MotorLogConfig(
                    position = true,
                    statorCurrent = false,
                    current = false,
                    velocity = false,
                    absoluteEncoder = false,
                    voltage = true
                )
        )

    private val positionTorque = PositionTorqueCurrentFOC(0.0)

    private var setpoint = ClimbLevels.RETRACTED

    val atSetpoint = Trigger {
        setpoint.height.isNear(motor.inputs.distance, TOLERANCE)
    }

    override fun setTarget(value: ClimbLevels): Command = runOnce {
        setpoint = value
        motor.setControl(
            positionTorque.withPosition(
                value.height.toAngle(SPROCKET_DIAMETER, GEAR_RATIO)
            )
        )
    }

    override fun periodic() {
        motor.periodic()
        Logger.recordOutput("Subsystems/Climb/setpoint", setpoint)
        Logger.recordOutput("Subsystems/Climb/atSetpoint", setpoint)
    }
}
