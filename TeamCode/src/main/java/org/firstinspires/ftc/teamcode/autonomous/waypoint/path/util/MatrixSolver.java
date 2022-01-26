package org.firstinspires.ftc.teamcode.autonomous.waypoint.path.util;

public class MatrixSolver {
    //Represents each diagonal in tridiagonal matrix.
    public float[] a, b, c;

    public MatrixSolver(int size)
    {
        a = new float[size];
        b = new float[size];
        c = new float[size];
    }

    //Returns the width/height of matrix.
    private int getMatrixSize()
    {
        if (a != null)
            return a.length;
        else
            return 0;
    }

    //Solve tridiagonal matrix, where d is the right side of the equation. This employs the Thomas algorithm.
    public float[] solveMatrix(float[] d) throws Exception {
        int size = getMatrixSize();

        if (size != d.length)
            throw new Exception("The input supplied is not the same size as the matrix.");


        float[] cPrime = new float[size];
        cPrime[0] = c[0] / b[0];

        for (int i = 1; i < size; i++)
        {
            cPrime[i] = c[i] / (b[i] - (a[i] * cPrime[i - 1]));
        }

        float[] dPrime = new float[size];
        dPrime[0] = d[0] / b[0];

        for (int i = 1; i < size; i++)
        {
            dPrime[i] = (d[i] - (a[i] * dPrime[i-1])) / (b[i] - (a[i] * cPrime[i-1]));
        }

        //Backwards substitution

        float[] x = new float[size];
        x[size - 1] = dPrime[size - 1];

        for (int i = size - 2; i >= 0; i--)
        {
            x[i] = dPrime[i] - (cPrime[i] * x[i + 1]);
        }

        return x;
    }


}
