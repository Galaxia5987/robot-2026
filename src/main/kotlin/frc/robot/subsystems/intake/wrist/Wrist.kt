package frc.robot.subsystems.intake.wrist

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.universal_motor.UniversalTalonFX

object Wrist : SubsystemBase() {
    private val motor =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            simGains = SIM_GAINS,
            gearRatio = GEAR_RATIO
        )

    override fun WristPositionsCommandFactory(){

    }
}
