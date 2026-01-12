package frc.robot.lib;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class GeneralRobotLoop extends SubsystemBase {
    private static final List<Runnable> toRun = new ArrayList<>();

    public static void register(Runnable... loops) {
        toRun.addAll(Arrays.stream(loops).toList());
    }

    @Override
    public void periodic() {
        toRun.forEach(Runnable::run);
    }
}
