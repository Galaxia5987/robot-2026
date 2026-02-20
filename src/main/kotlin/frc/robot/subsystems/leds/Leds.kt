package frc.robot.subsystems.leds


import com.ctre.phoenix6.controls.SolidColor
import com.ctre.phoenix6.hardware.CANdle
import com.ctre.phoenix6.signals.RGBWColor
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.rps
import frc.robot.states.shooting.shooterAtSetpoint
import frc.robot.states.shooting.shooting
import frc.robot.subsystems.sensors.Sensors
import frc.robot.subsystems.shooter.flywheel.Flywheel


object  Leds : SubsystemBase(){
val solidColorRequest = SolidColor(8, 399)
    private val candle = CANdle(0)

 private  fun setColor(color: RGBWColor): Command = run {
     candle.setControl(solidColorRequest.withColor(color))
    }
    private fun yellow(): Command = runOnce{
   setColor(RGBWColor.fromHSV(58.0 , 100.0 ,100.0))
    }
    private fun red(): Command = runOnce{
        setColor(RGBWColor.fromHSV(0.0 , 100.0 , 100.0))
    }
    val isLoading = Trigger(Sensors.cantCloseIntake).onTrue(yellow())
    val isShooting = Trigger({Flywheel.inputs.velocity>0.rps}).onTrue(red())



}


