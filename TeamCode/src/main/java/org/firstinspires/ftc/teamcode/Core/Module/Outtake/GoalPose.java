package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

public class GoalPose {
    public double x;
    public double y;
    public double targetZ;
    public double frontWallZ;
    public double frontWallOffsetRadius;

    public GoalPose(double x, double y, double targetZ, double frontWallZ, double frontWallOffsetRadius) {
        this.x = x;
        this.y = y;
        this.targetZ = targetZ;
        this.frontWallZ = frontWallZ;
        this.frontWallOffsetRadius = frontWallOffsetRadius;
    }
}