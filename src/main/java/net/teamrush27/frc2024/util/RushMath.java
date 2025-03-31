package net.teamrush27.frc2024.util;

public class RushMath {

    public static double pythagorean(double a, double b){
        return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
    }

    public static double pythagorean(double a, double b, double c) {
        return Math.sqrt(a * a + b * b + c * c);
    }
}
