package frc.robot.subsystems.leds

import com.ctre.phoenix6.controls.RainbowAnimation
import com.ctre.phoenix6.controls.SolidColor
import com.ctre.phoenix6.controls.TwinkleAnimation
import com.ctre.phoenix6.hardware.CANdle
import com.ctre.phoenix6.signals.RGBWColor
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotContainer
import frc.robot.states.intaking.IntakingStates
import frc.robot.states.setpoints_manager.SetpointsManager.isShootingOnMove
import frc.robot.states.shooting.ShootingState
import frc.robot.states.shooting.allSubsystemsAtSetpoint
import frc.robot.subsystems.intake.roller.Roller
import frc.robot.subsystems.sensors.Sensors
import frc.robot.subsystems.shooter.flywheel.Flywheel

object LEDSubsystem : SubsystemBase() {

    private enum class LedMode {
        SHOOTING_ON_MOVE,
        SHOOTING,
        PRIMING,
        CANT_SHOOT,
        PUMPING,
        INTAKING,
        HAS_FUEL,
        EMPTY,
    }

    private val candle = CANdle(39)
    private val solidColorRequest = SolidColor(startIndex, endIndex)
    private var currentMode: LedMode? = null

    private fun setSolidColor(color: RGBWColor) {
        candle.setControl(solidColorRequest.withColor(color))
    }

    private fun activateFlickering() {
        candle.setControl(TwinkleAnimation(startIndex, endIndex))
    }

    private fun setRainbow() {
        candle.setControl(RainbowAnimation(startIndex, endIndex))
    }

    private fun setFlicker(color: RGBWColor) {
        setSolidColor(color)
        activateFlickering()
    }

    private fun desiredMode(): LedMode =
        when {
            isShootingOnMove.asBoolean -> LedMode.SHOOTING_ON_MOVE

            ShootingState.SHOOTING.trigger.asBoolean -> LedMode.SHOOTING

            RobotContainer.shooting?.cantShootToHub?.asBoolean == true ->
                LedMode.CANT_SHOOT

            ShootingState.PRIMING.trigger.asBoolean -> LedMode.PRIMING

            IntakingStates.PUMPING.trigger.asBoolean -> LedMode.PUMPING

            IntakingStates.INTAKING.trigger.asBoolean -> LedMode.INTAKING

            Sensors.hasFuel.asBoolean || Roller.isActive.asBoolean ->
                LedMode.HAS_FUEL

            else -> LedMode.EMPTY
        }

    private fun setMode(mode: LedMode) {
        if (mode == currentMode) return
        currentMode = mode
        when (mode) {
            LedMode.SHOOTING_ON_MOVE -> setRainbow()
            LedMode.SHOOTING -> setSolidColor(RED)
            LedMode.CANT_SHOOT -> setSolidColor(ORANGE)
            LedMode.PRIMING -> setFlicker(RED)
            LedMode.PUMPING -> setFlicker(BLUE)
            LedMode.INTAKING -> setSolidColor(BLUE)
            LedMode.HAS_FUEL -> setSolidColor(YELLOW)
            LedMode.EMPTY -> setSolidColor(WHITE)
        }
    }

    override fun periodic() {
        setMode(desiredMode())
    }
}
