package frc.robot.subsystems.vision.supplier;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.Optional;
import java.util.function.Supplier;


public class NonNullablePoseSupplier implements Supplier<Pose2d> {
    Supplier<Pose2d> rawSupplier;
    Pose2d lastNonNullMeasurement;
    public NonNullablePoseSupplier(Supplier<Pose2d> rawSupplier) {
        this.rawSupplier = rawSupplier;
    }

    /**
     * Gets a result.
     *
     * @return a result
     */
    @Override
    public Pose2d get() {
        try {
            lastNonNullMeasurement = Optional.ofNullable(rawSupplier.get()).orElse(lastNonNullMeasurement);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        return lastNonNullMeasurement;
    }
}
