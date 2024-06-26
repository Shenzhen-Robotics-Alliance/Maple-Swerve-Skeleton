package frc.robot.HardwareIO.Helpers;

import java.util.HashSet;
import java.util.Set;

public final class PeriodicallyUpdatedInputs {
    private static final Set<PeriodicallyUpdatedInput> inputs = new HashSet<>();
    public interface PeriodicallyUpdatedInput {
        void update();
    }

    public static void register(PeriodicallyUpdatedInput input) {
        inputs.add(input);
    }
    public static void cancelRegister(PeriodicallyUpdatedInput input) {
        inputs.remove(input);
    }

    public static void updateInputs() {
        for (PeriodicallyUpdatedInput input:inputs)
            input.update();
    }
}
