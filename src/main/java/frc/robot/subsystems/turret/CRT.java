package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import java.util.OptionalDouble;

/** Utility for reconstructing a continuous turret angle from two absolute encoders using CRT. */
public final class CRT {
  private CRT() {}

  public record Parameters(
      int modulusA,
      int modulusB,
      double encoderAOffsetRot,
      double encoderBOffsetRot,
      double turretZeroOffsetDeg,
      double minAngleDeg,
      double maxAngleDeg) {}

  public static OptionalDouble solveAbsoluteAngleDeg(
      double encoderAAbsRot, double encoderBAbsRot, Parameters params) {
    if (!Double.isFinite(encoderAAbsRot)
        || !Double.isFinite(encoderBAbsRot)
        || !isValidParameters(params)) {
      return OptionalDouble.empty();
    }

    int residueA = toResidue(encoderAAbsRot, params.modulusA(), params.encoderAOffsetRot());
    int residueB = toResidue(encoderBAbsRot, params.modulusB(), params.encoderBOffsetRot());

    long combinedResidue = chineseRemainder(residueA, params.modulusA(), residueB, params.modulusB());
    if (combinedResidue < 0) {
      return OptionalDouble.empty();
    }

    long combinedModulus = (long) params.modulusA() * params.modulusB();
    if (combinedModulus <= 0) {
      return OptionalDouble.empty();
    }

    double angleDeg = combinedResidue * (360.0 / combinedModulus) + params.turretZeroOffsetDeg();
    double wrappedAngle = MathUtil.inputModulus(angleDeg, params.minAngleDeg(), params.maxAngleDeg());
    return Double.isFinite(wrappedAngle) ? OptionalDouble.of(wrappedAngle) : OptionalDouble.empty();
  }

  private static boolean isValidParameters(Parameters params) {
    return params != null
        && params.modulusA() > 1
        && params.modulusB() > 1
        && gcd(params.modulusA(), params.modulusB()) == 1
        && Double.isFinite(params.encoderAOffsetRot())
        && Double.isFinite(params.encoderBOffsetRot())
        && Double.isFinite(params.turretZeroOffsetDeg())
        && Double.isFinite(params.minAngleDeg())
        && Double.isFinite(params.maxAngleDeg())
        && params.minAngleDeg() < params.maxAngleDeg();
  }

  private static int toResidue(double encoderAbsRot, int modulus, double offsetRot) {
    double normalized = MathUtil.inputModulus(encoderAbsRot - offsetRot, 0.0, 1.0);
    long rawResidue = Math.round(normalized * modulus);
    return positiveMod((int) rawResidue, modulus);
  }

  private static long chineseRemainder(int a, int m1, int b, int m2) {
    long lM1 = m1;
    long lM2 = m2;
    long modulusProduct = lM1 * lM2;

    long n1 = modulusProduct / lM1;
    long n2 = modulusProduct / lM2;
    long inv1 = modInverse(n1, lM1);
    long inv2 = modInverse(n2, lM2);
    if (inv1 < 0 || inv2 < 0) {
      return -1;
    }

    long x = (a * n1 * inv1 + b * n2 * inv2) % modulusProduct;
    return positiveMod(x, modulusProduct);
  }

  private static long modInverse(long value, long modulus) {
    long t = 0;
    long newT = 1;
    long r = modulus;
    long newR = positiveMod(value, modulus);

    while (newR != 0) {
      long quotient = r / newR;

      long tmpT = newT;
      newT = t - quotient * newT;
      t = tmpT;

      long tmpR = newR;
      newR = r - quotient * newR;
      r = tmpR;
    }

    if (r > 1) {
      return -1;
    }
    return positiveMod(t, modulus);
  }

  private static int gcd(int a, int b) {
    int x = Math.abs(a);
    int y = Math.abs(b);
    while (y != 0) {
      int t = x % y;
      x = y;
      y = t;
    }
    return x;
  }

  private static int positiveMod(int value, int modulus) {
    int modded = value % modulus;
    return modded >= 0 ? modded : modded + modulus;
  }

  private static long positiveMod(long value, long modulus) {
    long modded = value % modulus;
    return modded >= 0 ? modded : modded + modulus;
  }
}
