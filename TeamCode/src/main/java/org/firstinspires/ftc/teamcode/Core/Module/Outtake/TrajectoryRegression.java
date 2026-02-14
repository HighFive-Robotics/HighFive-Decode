package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

public class TrajectoryRegression {

    private static final double downA = 5.02502747733E-33;
    private static final double downB = -2.32053461186E-29;
    private static final double downC = 4.49913375867E-26;
    private static final double downD = -5.00288776291E-23;
    private static final double downF = 3.61655927294E-20;
    private static final double downG = -1.81220201039E-17;
    private static final double downH = 6.52226974479E-15;
    private static final double downI = -1.71876400232E-12;
    private static final double downJ = 3.34372682813E-10;
    private static final double downK = -4.80011885576E-8;
    private static final double downL = 0.00000504119816252;
    private static final double downM = -0.000380358115941;
    private static final double downN = 0.0199702203189;
    private static final double downO = -0.689788406172;
    private static final double downP = 14.0482217018;
    private static final double downQ = -125.004841328;
    private static final double upA = -9.201304940118E-33;
    private static final double upB = 3.128330393841E-29;
    private static final double upC = -4.843882638944E-26;
    private static final double upD = 4.520129398433E-23;
    private static final double upF = -2.835116647496E-20;
    private static final double upG = 1.26219620553E-17;
    private static final double upH = -4.10643755153E-15;
    private static final double upI = 9.904339960864E-13;
    private static final double upJ = -1.778240917699E-10;
    private static final double upK = 2.366205747903E-8;
    private static final double upL = -0.000002303780517619;
    private static final double upM = 0.0001604044564653;
    private static final double upN = -0.007694804578658;
    private static final double upO = 0.2390664657819;
    private static final double upP = -4.289508500321;
    private static final double upQ = 33.74292059678;

    public static double calculateDown(double x) {
        return downQ + x * (downP + x * (downO + x * (downN + x * (downM + x * (downL + x * (downK + x * (downJ + x * (downI + x * (downH + x * (downG + x * (downF + x * (downD + x * (downC + x * (downB + x * downA))))))))))))));
    }

    public static double calculateUp(double x) {
        return upQ + x * (upP + x * (upO + x * (upN + x * (upM + x * (upL + x * (upK + x * (upJ + x * (upI + x * (upH + x * (upG + x * (upF + x * (upD + x * (upC + x * (upB + x * upA))))))))))))));
    }
}