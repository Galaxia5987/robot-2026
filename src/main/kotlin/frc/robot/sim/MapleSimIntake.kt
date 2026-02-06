package frc.robot.sim

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.IS_SIM
import frc.robot.driveSimulation
import frc.robot.lib.extensions.cm
import frc.robot.states.intaking.IntakingStates
import frc.robot.subsystems.sensors.Sensors
import org.ironmaple.simulation.IntakeSimulation
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean

class MapleSimIntake {
    private val intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveSimulation,
            80.cm,
            30.cm,
            IntakeSimulation.IntakeSide.FRONT,
            55
        )

    val ballCount: Int
        get() = intakeSimulation.gamePiecesAmount

    val useIntakeCounting: LoggedNetworkBoolean = LoggedNetworkBoolean("/Tuning/Sensors/useIntakeCounting", true)

    private val intakeMapleSim: Trigger =
        IntakingStates.INTAKING.trigger
            .and(IS_SIM)
            .onTrue(Commands.runOnce({ intakeSimulation.startIntake() }))
            .onFalse(Commands.runOnce({ intakeSimulation.stopIntake() }))

    fun obtainBallFromIntake() = intakeSimulation.obtainGamePieceFromIntake()

}
