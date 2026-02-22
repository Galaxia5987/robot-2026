import com.ctre.phoenix6.controls.SolidColor
import com.ctre.phoenix6.hardware.CANdle
import com.ctre.phoenix6.signals.RGBWColor
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.not
import frc.robot.lib.extensions.rps
import frc.robot.subsystems.intake.roller.Roller
import frc.robot.subsystems.sensors.Sensors
import frc.robot.subsystems.shooter.flywheel.Flywheel

/*
    Robot is full - Flickering Orange
    Robot is not full - Flickering Yellow
    Robot is empty - Static White
    Robot is shooting - Static Red
    Robot is loading - Static Yellow
 */

object LEDSubsystem : SubsystemBase() {
    private val candle = CANdle(0)
    private val solidColorRequest = SolidColor(8, 399)

    private fun setColor (color: RGBWColor) : Command = runOnce {
        candle.setControl(solidColorRequest.withColor(color))
    }

    private fun flickeringOrange () : Command = run {
        setColor(RGBWColor.fromHSV(15.0, 100.0, 100.0))
            .withTimeout(0.75)
        setColor(RGBWColor.fromHSV(0.0, 0.0, 0.0))
            .withTimeout(0.75)
    }

    private fun flickeringYellow() : Command = runOnce {
        setColor(RGBWColor.fromHSV(50.0, 100.0, 100.0))
            .withTimeout(0.75)
        setColor(RGBWColor.fromHSV(0.0, 0.0, 0.0))
            .withTimeout(0.75)
    }

    private fun staticWhite() : Command = runOnce {
        setColor(RGBWColor.fromHSV(0.0, 0.0, 100.0))
    }

    private fun staticRed() : Command = runOnce {
        setColor(RGBWColor.fromHSV(0.0, 100.0, 100.0))
    }

    private fun staticYellow() : Command = runOnce {
        setColor(RGBWColor.fromHSV(50.0, 100.0, 100.0))
    }

    val rollerOn = Trigger{ Roller.inputs.velocity > 0.rps }
    val flywheelOn = Trigger{ Flywheel.inputs.velocity > 0.rps }

    init {
        val isFull = Sensors.isFull.onTrue(flickeringOrange())
        val isEmpty = Sensors.hasFuel.onFalse(staticWhite())
        val isShooting =  flywheelOn.onTrue(staticRed())
        val isLoading = rollerOn.and(!Sensors.isFull).onTrue(staticYellow())
        val isNotFull = !isLoading.and(!Sensors.isFull).and(!isEmpty).onTrue(flickeringYellow())
    }
}