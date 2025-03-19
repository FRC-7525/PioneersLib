package org.team7525.coefficients;

public record PIDCoefficients(
        double p,
        double i,
        double d,
        double iZone
) {}
