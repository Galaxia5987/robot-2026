package frc.robot.lib

import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.littletonrobotics.junction.AutoLogOutput

const val ABNORMAL_EVENT_NAME = "### [Error] DRIVER PROBLEM REPORTED ###"

@AutoLogOutput private var marked = false

private fun markEvent(eventName: String): Command =
    Commands.sequence(
        Commands.runOnce({
            DataLogManager.log(eventName)
            marked = true
        }),
        Commands.waitSeconds(5.0),
        Commands.runOnce({ marked = false })
    )

fun markAbnormalEvent(): Command = markEvent(ABNORMAL_EVENT_NAME)
