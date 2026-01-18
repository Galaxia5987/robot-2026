package frc.robot.subsystems.intake.roller

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.subsystems.roller.AUX_MOTOR_PORT
import frc.robot.subsystems.roller.MAIN_MOTOR_PORT
import frc.robot.subsystems.roller.MOTOR_CONFIG
import frc.robot.subsystems.roller.RollerPositions
import frc.robot.subsystems.roller.RollerPositionsCommandFactory

object Roller : SubsystemBase(), RollerPositionsCommandFactory {
    private val motor =
        UniversalTalonFX(
            port = MAIN_MOTOR_PORT,
            config = MOTOR_CONFIG,
            subsystem = name
        )

    private val auxMotor =
        UniversalTalonFX(
                port = AUX_MOTOR_PORT,
                config = MOTOR_CONFIG,
                subsystem = name
            )
            .apply {
                setControl(
                    Follower(MAIN_MOTOR_PORT, MotorAlignmentValue.Aligned)
                )
            }

    private val voltageRequest = VoltageOut(0.0)

    override fun setTarget(value: RollerPositions): Command = runOnce {
        motor.setControl(voltageRequest.withOutput(value.voltage))
    }

    override fun periodic() {
        motor.periodic()
    }
}
