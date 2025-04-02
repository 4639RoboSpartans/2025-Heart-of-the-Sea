package frc.lib.annotation;

import frc.robot.subsystemManager.Subsystems;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Indicates that a method is meant only to be called by the {@link Subsystems} class and no other.
 * For other cases, use the corresponding method in {@link Subsystems}.
 */

@Target(ElementType.METHOD)
@Retention(RetentionPolicy.SOURCE)
public @interface ForSubsystemManagerUseOnly {
}
