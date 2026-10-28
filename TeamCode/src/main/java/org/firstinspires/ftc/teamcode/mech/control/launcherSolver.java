package org.firstinspires.ftc.teamcode.mech.control;
public class launcherSolver
{
    private final double optiv = 67; //Optimal horizontal velocity upon entry is m/s prob
    private final double pRad = 6.7; //Radius of pollen in cm
    private final double nRad = 6.7; //Radius of nectar in cm
    private final double balRight = 6.7; //Radius of rubber thingy in cm
    private final double hdtl = 6.7; //Horizontal distance from limelight to launcher
    private final double vdtl = 6.7; //Vertical distance from limelight to launcher
    public double pollenAngle(double thd, double tvd)
    //thd - horizontal distance from limelight to obelisk
    //tvd - vertical distance from limelight to obelisk
    {
        return Math.atan((tvd+pRad+vdtl)/thd + 0.5*9.8*(thd+hdtl)/(optiv*optiv));
    }
    public double nectarAngle(double thd, double tvd)
    {
        return Math.atan((tvd+nRad+vdtl)/thd + 0.5*9.8*(thd+hdtl)/(optiv*optiv));
    }
    public double pollenRPM(double thd, double tvd)
    {
        return 60*optiv/(Math.cos(pollenAngle(thd, tvd))*(balRight + pRad)); //v*cos = v-horizontal, v = w*r
    }
    public double nectarRPM(double thd, double tvd)
    {
        return 60*optiv/(Math.cos(nectarAngle(thd, tvd))*(balRight + nRad));
    }

}
