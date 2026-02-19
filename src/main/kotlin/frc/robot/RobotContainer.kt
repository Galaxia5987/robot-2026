package frc.robot

import choreo.Choreo
import choreo.util.ChoreoAllianceFlipUtil
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.autonomous.Test
import frc.robot.lib.Mode
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.states.intaking.IntakingStates
import frc.robot.states.setpoints_manager.aimingSetpoint
import frc.robot.states.shooting.Shooting
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.turret.Turret
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

object RobotContainer {
    private val driverController = CommandPS5Controller(0)
    private val autoChooser: LoggedDashboardChooser<Command>

    init {
        drive // Ensure Drive is initialized
        autoChooser =
            LoggedDashboardChooser(
                "Auto Choices",
                AutoBuilder.buildAutoChooser()
            )

        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()

        if (CURRENT_MODE == Mode.SIM) {
            SimulatedArena.getInstance()
                .addDriveTrainSimulation(driveSimulation)
            SimulatedArena.getInstance().resetFieldForAuto()
        }

        enableAutoLogOutputFor(this)
    }

    private fun configureDefaultCommands() {
        drive.defaultCommand =
            DriveCommands.joystickDrive(
                { -driverController.leftY },
                { -driverController.leftX },
                { -driverController.rightX * 1.2 }
            )

        Turret.defaultCommand = Turret.setAngle { Turret.aimingSetpoint() }
        Hood.defaultCommand = Hood.setAngle { Hood.aimingSetpoint() }
    }

    private fun configureButtonBindings() {
        driverController.create().onTrue(DriveCommands.resetGyro())

        // Intake Bindings
        val intakeButton = driverController.R2()

        intakeButton.onTrue(IntakingStates.INTAKING.set())
        intakeButton.negate().onTrue(IntakingStates.CLOSED.set())

        Shooting(driverController.L2())
    }

    fun getAutonomousCommand(): Command = autoChooser.get()

    private fun registerAutoCommands() {
        // SysIds
        autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization()
        )
        autoChooser.addOption(
            "Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterization()
        )
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )

        autoChooser.addOption(
            "swerveFFCharacterization",
            DriveCommands.feedforwardCharacterization()
        )
        autoChooser.addOption(
            "Test", Test()
        )

//        autoChooser.addOption("Test", Test())
    }
}
