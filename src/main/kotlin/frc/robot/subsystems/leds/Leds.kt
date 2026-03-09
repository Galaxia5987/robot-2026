package frc.robot.subsystems.leds

import com.ctre.phoenix6.controls.RainbowAnimation
import com.ctre.phoenix6.controls.SolidColor
import com.ctre.phoenix6.controls.StrobeAnimation
import com.ctre.phoenix6.controls.TwinkleAnimation
import com.ctre.phoenix6.hardware.CANdle
import com.ctre.phoenix6.signals.RGBWColor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotContainer
import frc.robot.states.intaking.IntakingStates
import frc.robot.states.setpoints_manager.SetpointsManager.isShootingOnMove
import frc.robot.states.shooting.ShootingState

object LEDSubsystem : SubsystemBase() {

    private enum class LedMode {
        DISABLED,
        SHIFT_END,
        SHOOTING_ON_MOVE,
        SHOOTING,
        PRIMING,
        CANT_SHOOT,
        PUMPING,
        INTAKING,
        EMPTY,
    }

    private val candle = CANdle(39)
    private val solidColorRequest = SolidColor(startIndex, endIndex)
    private val flickerRequest = StrobeAnimation(startIndex, endIndex)
    private val rainbowRequest = RainbowAnimation(startIndex, endIndex)
    private var currentMode: LedMode? = null

    private fun setSolidColor(color: RGBWColor) {
        candle.setControl(solidColorRequest.withColor(color))
    }

    private fun setRainbow() {
        candle.setControl(rainbowRequest)
    }

    private fun setFlicker(color: RGBWColor) {
        candle.setControl(flickerRequest.withColor(color))

    }

    private fun desiredMode(): LedMode =
        when {

//            timeLeftForShift <= 3.sec -> LedMode.SHIFT_END

            DriverStation.isDisabled() -> LedMode.DISABLED

            RobotContainer.shooting?.cantShootToHub?.asBoolean == true ->
                LedMode.CANT_SHOOT

            isShootingOnMove.asBoolean && ShootingState.SHOOTING.trigger.asBoolean -> LedMode.SHOOTING_ON_MOVE

            ShootingState.SHOOTING.trigger.asBoolean -> LedMode.SHOOTING


            ShootingState.PRIMING.trigger.asBoolean -> LedMode.PRIMING

            IntakingStates.PUMPING.trigger.asBoolean -> LedMode.PUMPING


            IntakingStates.INTAKING.trigger.asBoolean -> LedMode.INTAKING

            else -> LedMode.EMPTY
        }

    private fun setMode(mode: LedMode) {
        if (mode == currentMode) return
        currentMode = mode
        when (mode) {
            LedMode.DISABLED -> setRainbow()
            LedMode.CANT_SHOOT -> setFlicker(RED)
            LedMode.SHIFT_END -> setFlicker(WHITE)
            LedMode.SHOOTING_ON_MOVE -> setRainbow()
            LedMode.SHOOTING -> setFlicker(GREEN)
            LedMode.PRIMING -> setSolidColor(YELLOW)
            LedMode.PUMPING -> setFlicker(BLUE)
            LedMode.INTAKING -> setSolidColor(BLUE)
            LedMode.EMPTY -> setSolidColor(WHITE)
        }
    }

    override fun periodic() {
        setMode(desiredMode())
    }
}
