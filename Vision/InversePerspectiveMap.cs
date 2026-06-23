namespace RobloxRouteBot.Vision;

/// <summary>
/// Inverse Perspective Mapping: разворачивает КОСОЙ (наклонённый) вид земли в виртуальный top-down.
/// Наклонённый и строго-верхний виды с общим оптическим центром связаны гомографией чистого пита
/// H = K·R(θ)·K⁻¹, где f = (N/2)/tan(FOVv/2). Высота камеры и нормаль земли сокращаются — нужны
/// только FOV и угол наклона, без покадровой калибровки. На ровной земле ректифицированный кадр
/// меняется кадр-к-кадру как ~поворот+сдвиг — ровно та модель, что ловит Fourier-Mellin.
///
/// θ=0 → identity (быстрый pass-through = текущее поведение). Над горизонтом / вне кадра → 0.
/// Чистый managed, LUT как у ImageWarp.LogPolar; ~4 MAC/пиксель, доли мс.
/// </summary>
public sealed class InversePerspectiveMap
{
    private int _n;
    private bool _identity = true;
    private int[]? _ix0, _iy0;
    private float[]? _wx, _wy;
    private bool[]? _ok;
    private double[] _h = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

    public bool Identity => _identity;
    public IReadOnlyList<double> Homography => _h;

    public void Build(int n, float fovVRad, float tiltRad)
    {
        _n = n;
        if (MathF.Abs(tiltRad) < 1e-4f) { _identity = true; return; }
        _identity = false;
        EnsureBuffers(n);

        float f = (n / 2f) / MathF.Tan(fovVRad / 2f);
        float cx = n / 2f, cy = n / 2f;
        _h = ComputeH(f, cx, cy, tiltRad);
        var H = _h;

        for (int oy = 0; oy < n; oy++)
        {
            for (int ox = 0; ox < n; ox++)
            {
                double wx = H[0] * ox + H[1] * oy + H[2];
                double wy = H[3] * ox + H[4] * oy + H[5];
                double ww = H[6] * ox + H[7] * oy + H[8];
                int idx = oy * n + ox;
                if (ww <= 1e-6) { _ok![idx] = false; continue; } // за горизонтом
                double sx = wx / ww, sy = wy / ww;
                if (sx < 0 || sy < 0 || sx > n - 1 || sy > n - 1) { _ok![idx] = false; continue; }
                int x0 = (int)sx, y0 = (int)sy;
                _ix0![idx] = x0; _iy0![idx] = y0;
                _wx![idx] = (float)(sx - x0); _wy![idx] = (float)(sy - y0);
                _ok![idx] = true;
            }
        }
    }

    public void Rectify(GrayFrame src, GrayFrame dst)
    {
        if (_identity) { Array.Copy(src.Pixels, dst.Pixels, src.Pixels.Length); return; }
        int n = _n;
        byte[] sp = src.Pixels, dp = dst.Pixels;
        for (int i = 0; i < n * n; i++)
        {
            if (!_ok![i]) { dp[i] = 0; continue; }
            int x0 = _ix0![i], y0 = _iy0![i];
            int x1 = Math.Min(x0 + 1, n - 1), y1 = Math.Min(y0 + 1, n - 1);
            float wx = _wx![i], wy = _wy![i];
            float a = sp[y0 * n + x0], b = sp[y0 * n + x1], c = sp[y1 * n + x0], d = sp[y1 * n + x1];
            float top = a + (b - a) * wx, bot = c + (d - c) * wx;
            dp[i] = (byte)Math.Clamp(top + (bot - top) * wy, 0f, 255f);
        }
    }

    private void EnsureBuffers(int n)
    {
        if (_ix0 != null && _ix0.Length == n * n) return;
        _ix0 = new int[n * n];
        _iy0 = new int[n * n];
        _wx = new float[n * n];
        _wy = new float[n * n];
        _ok = new bool[n * n];
    }

    /// <summary>H = K·R_x(θ)·K⁻¹ (3×3, row-major). Отображает dst(top-down) → src(наклонённый).</summary>
    private static double[] ComputeH(float f, float cx, float cy, float theta)
    {
        double c = Math.Cos(theta), s = Math.Sin(theta);
        double[] K = { f, 0, cx, 0, f, cy, 0, 0, 1 };
        double[] R = { 1, 0, 0, 0, c, -s, 0, s, c };
        double[] Ki = { 1.0 / f, 0, -cx / f, 0, 1.0 / f, -cy / f, 0, 0, 1 };
        return Mul3(Mul3(K, R), Ki);
    }

    private static double[] Mul3(double[] a, double[] b)
    {
        var m = new double[9];
        for (int r = 0; r < 3; r++)
            for (int col = 0; col < 3; col++)
                m[r * 3 + col] = a[r * 3 + 0] * b[0 * 3 + col]
                               + a[r * 3 + 1] * b[1 * 3 + col]
                               + a[r * 3 + 2] * b[2 * 3 + col];
        return m;
    }
}
