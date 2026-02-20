import com.ctre.phoenix6.controls.SolidColor
import com.ctre.phoenix6.hardware.CANdle
import com.ctre.phoenix6.signals.RGBWColor
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.rps
import frc.robot.subsystems.sensors.Sensors
import frc.robot.subsystems.shooter.flywheel.Flywheel

    private val candle = CANdle(0)

        candle.setControl(solidColorRequest.withColor(color))
    }
    }
        setColor(RGBWColor.fromHSV(0.0, 100.0, 100.0))
    }


}
