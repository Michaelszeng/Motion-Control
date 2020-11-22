package org.firstinspires.ftc.teamcode.util.MathFunctions;

public class Quadratic {
    public double nRoots;
    public double x1, x2;

    public void solve(double a, double b, double c) {

        double determinant = b * b - 4 * a * c;

        // condition for real and different roots
        if(determinant > 0) {
            nRoots = 2;
            x1 = (-b + Math.sqrt(determinant)) / (2 * a);
            x2 = (-b - Math.sqrt(determinant)) / (2 * a);

//            System.out.format("root1 = %.2f and root2 = %.2f", root1 , root2);
        }
        // Condition for real and equal roots
        else if(determinant == 0) {
            nRoots = 1;
            x1 = x2 = -b / (2 * a);

//            System.out.format("root1 = root2 = %.2f;", root1);
        }
        // If roots are not real
        else {
            nRoots = 0;
            double realPart = -b / (2 *a);
            double imaginaryPart = Math.sqrt(-determinant) / (2 * a);

            x1 = Double.NaN;
            x2 = Double.NaN;
//            System.out.format("root1 = %.2f+%.2fi and root2 = %.2f-%.2fi", realPart, imaginaryPart, realPart, imaginaryPart);
        }
    }
}