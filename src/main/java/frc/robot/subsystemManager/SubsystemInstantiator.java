package frc.robot.subsystemManager;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.annotation.PackagePrivate;
import frc.robot.robot.Robot;

import java.util.function.Supplier;

/**
 * A helper class that subsystems can use to tell the SubsystemManager
 * how to instantiate them.
 */
public final class SubsystemInstantiator<T extends Subsystem> {
    private final Supplier<T> ifReal, ifSim, ifDummy;
    private final boolean useDummy, useSimOverride;

    @PackagePrivate
    T instantiateSubsystem() {
        if (useDummy) return ifDummy.get();
        if (useSimOverride) return ifSim.get();

        return Robot.isReal()
            ? ifReal.get()
            : ifSim.get();
    }

    public SubsystemInstantiator(Supplier<T> constructor) {
        this(constructor, constructor, constructor, false, false);
    }

    public SubsystemInstantiator(Supplier<T> ifReal, Supplier<T> ifSim) {
        this(ifReal, ifSim, ifSim, false, false);
    }

    /**
     * @param useSimOverride If true, the sim constructor will be used
     *                       regardless of whether the robot is real
     */
    public SubsystemInstantiator(Supplier<T> ifReal, Supplier<T> ifSim, boolean useSimOverride) {
        this(ifReal, ifSim, ifSim, false, useSimOverride);
    }

    /**
     * @param useDummy If true, the dummy constructor will ALWAYS be used
     */
    public SubsystemInstantiator(Supplier<T> ifReal, Supplier<T> ifSim, Supplier<T> ifDummy, boolean useDummy) {
        this(ifReal, ifSim, ifDummy, useDummy, false);
    }

    /**
     * @param useDummy       If true, the dummy constructor will ALWAYS be used
     * @param useSimOverride If true, the sim constructor will be used
     *                       regardless of whether the robot is real,
     *                       unless useDummy is set to true
     */
    public SubsystemInstantiator(
        Supplier<T> ifReal, Supplier<T> ifSim, Supplier<T> ifDummy,
        boolean useDummy, boolean useSimOverride
    ) {
        this.ifReal = ifReal;
        this.ifSim = ifSim;
        this.ifDummy = ifDummy;
        this.useDummy = useDummy;
        this.useSimOverride = useSimOverride;
    }
}
