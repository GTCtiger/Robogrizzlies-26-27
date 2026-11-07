package org.firstinspires.ftc.teamcode.mech.control;
public class launcherSolver
{
    private final double optiv = 216.1;
    private final double pRad = 3.556;
    private final double nRad = 4.572;
    private final double flywheel = 3.6;
    private final double hdtl = 30.48;
    private final double vdtl = 5;

    public double pollenAngle(double thd, double tvd) {
        return Math.atan(
                (tvd + pRad + vdtl) / (thd + hdtl)
                        + 0.5 * 981 * (thd + hdtl) / (optiv * optiv)
        );
    }

    public double nectarAngle(double thd, double tvd) {
        return Math.atan(
                (tvd + nRad + vdtl) / (thd + hdtl)
                        + 0.5 * 981 * (thd + hdtl) / (optiv * optiv)
        );
    }

    public double pollenRPM(double thd, double tvd) {
        return 60 * optiv /
                (2 * Math.PI * flywheel * Math.cos(pollenAngle(thd, tvd)));
    }

    public double nectarRPM(double thd, double tvd) {
        return 60 * optiv /
                (2 * Math.PI * flywheel * Math.cos(nectarAngle(thd, tvd)));
    }

}
