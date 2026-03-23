package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.autonomous.bullshitChallenge
import frc.robot.autonomous.depotDoubleCycle
import frc.robot.autonomous.depotMainShootOnMove
import frc.robot.autonomous.outpostMainShootOnMove
import frc.robot.lib.Mode
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.states.intaking.IntakingStates
import frc.robot.states.setpoints_manager.aimingSetpoint
import frc.robot.states.shooting.Shooting
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.leds.LEDSubsystem
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.turret.Turret
import kotlin.math.roundToInt
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

object RobotContainer {
    private val driverController = CommandPS5Controller(0)
    private val autoChooser: LoggedDashboardChooser<Command>

    var shooting: Shooting? = null

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
        LEDSubsystem
    }

    private fun configureDefaultCommands() {
        drive.defaultCommand =
            //            DriveCommands.joystickDrive(
            //                { -driverController.leftY },
            //                { -driverController.leftX },
            //                { -driverController.rightX },
            //            )
            DriveCommands.joystickDriveAtAngle(
                drive,
                { -driverController.leftY },
                { -driverController.leftX },
                { -driverController.rightX * 1.2 },
                {
                    val snappedLocation = drive.rotation.degrees / 90.0
                    Rotation2d.fromDegrees(snappedLocation.roundToInt() * 90.0)
                }
            )
        Turret.defaultCommand =
            Turret.setAngle(Turret.aimingSetpoint<Turret, () -> Angle>())
        Hood.defaultCommand =
            Hood.setAngle(Hood.aimingSetpoint<Hood, () -> Angle>())
    }

    private fun configureButtonBindings() {
        driverController.create().onTrue(DriveCommands.resetGyro())

        // Intake Bindings
        val intakeButton = driverController.R1()

        intakeButton.onTrue(IntakingStates.INTAKING.set())
        intakeButton.negate().onTrue(IntakingStates.CLOSED.set())

        shooting = Shooting(driverController.L2(), driverController.L1())

        driverController.povUp().whileTrue(Climb.up()).onFalse(Climb.stop())
        driverController.povDown().whileTrue(Climb.down()).onFalse(Climb.stop())
    }

    fun getAutonomousCommand(): Command = autoChooser.get()

    private fun registerAutoCommands() {
        autoChooser.addDefaultOption("Empty", Commands.none())

        autoChooser.addOption("bullshitChallenge", bullshitChallenge())

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

        autoChooser.addOption("ShootOnMoveTestPath", depotDoubleCycle())
        autoChooser.addOption("depotMainShootOnMove", depotMainShootOnMove())
        autoChooser.addOption(
            "outpostMainShootOnMove",
            outpostMainShootOnMove()
        )
        autoChooser.addOption("depotDoubleCycle", depotDoubleCycle())
    }
}
