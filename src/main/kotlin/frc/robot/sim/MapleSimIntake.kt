package frc.robot.sim

import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.IS_SIM
import frc.robot.driveSimulation
import frc.robot.lib.extensions.cm
import frc.robot.states.intaking.IntakingStates
import org.ironmaple.simulation.IntakeSimulation

class MapleSimIntake() {
    private val intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveSimulation,
            80.cm,
            30.cm,
            IntakeSimulation.IntakeSide.FRONT,
            55
        )

    val intakeMapleSim =
        IntakingStates.INTAKING.trigger
            .and(IS_SIM)
            .onTrue(Commands.runOnce({ intakeSimulation.startIntake() }))
            .onFalse(Commands.runOnce({ intakeSimulation.stopIntake() }))
}
