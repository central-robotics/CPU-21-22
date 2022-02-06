package org.firstinspires.ftc.teamcode.autonomous.nav.path.util.oldCode;

//CREDIT TO RYAN SEGHERS FOR A FAST INTERPRETATION OF THE CUBIC SPLINE.
public class Spline {
    private float[] a, b; //Coefficients
    private final float[] x;
    private final float[] y; //Points
    private float[] xOrig, yOrig; //Saved points
    private float firstSlope = Float.NaN;
    private float lastSlope = Float.NaN;
    private final float dist = 0;
    private int lastIndex;

    public Spline(float[] x, float[] y, float firstSlope, float lastSlope) throws Exception {
        this.x = x;
        this.y = y;
        this.firstSlope = firstSlope;
        this.lastSlope = lastSlope;
        computeSpline();
    }

    public Spline(float[] x, float[] y) throws Exception {
        this.x = x;
        this.y = y;
        computeSpline();
    }

    public float computeDerivative(float x) throws Exception {
        float qPrime;
        lastIndex = 0;

        int j = getNextXIndex(x);

        float dx = xOrig[j + 1] - xOrig[j];
        float dy = yOrig[j + 1] - yOrig[j];
        float t = (x - xOrig[j]) / dx;

        qPrime = dy / dx
                + (1 - 2 * t) * (a[j] * (1 - t) + b[j] * t) / dx
                + t * (1 - t) * (b[j] - a[j]) / dx;

        return qPrime;
    }

    private void computeSpline() throws Exception {
        if (Float.isInfinite(firstSlope) || Float.isInfinite(lastSlope))
            throw new Exception("Slopes cannot be infinite.");

        xOrig = x;
        yOrig = y;

        int n = x.length;
        float[] rNumbers = new float[n];

        MatrixSolver solver = new MatrixSolver(n);

        float dx1, dx2, dy1, dy2;

        if (Float.isNaN(firstSlope))
        {
            dx1 = x[1] - x[0];
            solver.c[0] = 1.0f / dx1;
            solver.b[0] = 2.0f * solver.c[0];
            rNumbers[0] = 3 * (y[1] - y[0]) / (dx1 * dx1);
        } else
        {
            solver.b[0] = 1;
            rNumbers[0] = firstSlope;
        }

        for (int i = 1; i < n - 1; i++)
        {
            dx1 = x[i] - x[i - 1];
            dx2 = x[i + 1] - x[i];

            solver.a[i] = 1.0f / dx1;
            solver.c[i] = 1.0f / dx2;
            solver.b[i] = 2.0f * (solver.a[i] + solver.c[i]);

            dy1 = y[i] - y[i - 1];
            dy2 = y[i + 1] - y[i];
            rNumbers[i] = 3 * (dy1 / (dx1 * dx1) + dy2 / (dx2 * dx2));
        }

        if (Float.isNaN(lastSlope))
        {
            dx1 = x[n - 1] - x[n - 2];
            dy1 = y[n - 1] - y[n - 2];
            solver.a[n - 1] = 1.0f / dx1;
            solver.b[n - 1] = 2.0f * solver.a[n - 1];
            rNumbers[n - 1] = 3 * (dy1 / (dx1 * dx1));
        }
        else
        {
            solver.b[n - 1] = 1;
            rNumbers[n - 1] = lastSlope;
        }

        float[] solution = solver.solveMatrix(rNumbers);

        a = new float[n-1];
        b = new float[n-1];

        for (int i = 1; i < n; i++)
        {
            dx1 = x[i] - x[i - 1];
            dy1 = y[i] - y[i - 1];
            a[i - 1] = solution[i - 1] * dx1 - dy1;
            b[i - 1] = -solution[i] * dx1 + dy1;
        }

    }

    public float[] mapSpline(float[] x) throws Exception {
        int n = x.length;
        float[] y = new float[n];

        lastIndex = 0;

        for (int i = 0; i < n; i++)
        {
            int j = getNextXIndex(x[i]);
            y[i] = evalSpline(x[i], j);
        }

        return y;
    }

    public float computeY(float x) throws Exception {
        lastIndex = 0;
        int nSpline = getNextXIndex(x);
        return evalSpline(x, nSpline);
    }

    private float evalSpline(float x, int j)
    {
        float dx = xOrig[j + 1] - xOrig[j];
        float t = (x - xOrig[j]) / dx;
        return (1 - t) * yOrig[j] + t * yOrig[j + 1] + t * (1 - t) * (a[j] * (1 - t) + b[j] * t);
    }

    private int getNextXIndex(float x) throws Exception {
        if (x < xOrig[lastIndex])
            throw new Exception("X out of order");

        while ((lastIndex < xOrig.length - 2) && (x > xOrig[lastIndex + 1]))
        {
            lastIndex++;
        }

        return lastIndex;
    }
}
