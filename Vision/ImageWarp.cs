namespace RobloxRouteBot.Vision;

/// <summary>
/// Геометрия для Fourier-Mellin: поворот кадра вокруг центра (билинейно) и лог-полярный ресэмпл
/// спектральной магнитуды (предрасчитанный LUT). Всё managed, тестируется отдельно.
/// </summary>
public static class ImageWarp
{
    /// <summary>Поворот серого кадра вокруг центра на angleRad (билинейно, за границей = 0).</summary>
    public static GrayFrame RotateAboutCenter(GrayFrame src, float angleRad)
    {
        int n = src.Width, h = src.Height;
        var dst = new GrayFrame(n, h);
        float c = MathF.Cos(angleRad), s = MathF.Sin(angleRad);
        float cx = n / 2f, cy = h / 2f;
        byte[] sp = src.Pixels, dp = dst.Pixels;

        for (int oy = 0; oy < h; oy++)
        {
            for (int ox = 0; ox < n; ox++)
            {
                float dx = ox - cx, dy = oy - cy;
                // обратное отображение: куда в исходнике смотрит выходной пиксель
                float fx = cx + c * dx + s * dy;
                float fy = cy - s * dx + c * dy;
                dp[oy * n + ox] = SampleBilinear(sp, n, h, fx, fy);
            }
        }
        return dst;
    }

    private static byte SampleBilinear(byte[] px, int w, int h, float x, float y)
    {
        if (x < 0 || y < 0 || x > w - 1 || y > h - 1) return 0;
        int x0 = (int)x, y0 = (int)y;
        int x1 = Math.Min(x0 + 1, w - 1), y1 = Math.Min(y0 + 1, h - 1);
        float fx = x - x0, fy = y - y0;
        float a = px[y0 * w + x0], b = px[y0 * w + x1], cc = px[y1 * w + x0], d = px[y1 * w + x1];
        float top = a + (b - a) * fx;
        float bot = cc + (d - cc) * fx;
        return (byte)Math.Clamp(top + (bot - top) * fy, 0f, 255f);
    }

    /// <summary>
    /// Лог-полярный ресэмпл: вход — центрированная (DC в центре) магнитуда n×n; выход n×n, где
    /// ось X = угол θ (0..2π), ось Y = log(r) (r от 1 до n/2). Сдвиг по X после фазовой корреляции = поворот.
    /// </summary>
    public sealed class LogPolar
    {
        private readonly int _n;
        private readonly int[] _ix0, _iy0;
        private readonly float[] _wx, _wy;

        public LogPolar(int n)
        {
            _n = n;
            _ix0 = new int[n * n];
            _iy0 = new int[n * n];
            _wx = new float[n * n];
            _wy = new float[n * n];

            float cx = n / 2f, cy = n / 2f;
            float maxR = n / 2f - 1f;
            float logMax = MathF.Log(maxR);

            for (int iy = 0; iy < n; iy++)
            {
                float rho = iy / (float)(n - 1) * logMax;
                float r = MathF.Exp(rho); // 1..maxR
                for (int ix = 0; ix < n; ix++)
                {
                    // θ по [0,π): |FFT| точечно-симметрична (период π), это даёт 2× разрешение по углу.
                    float theta = ix * (MathF.PI / n);
                    float u = cx + r * MathF.Cos(theta);
                    float v = cy + r * MathF.Sin(theta);
                    int u0 = (int)MathF.Floor(u), v0 = (int)MathF.Floor(v);
                    int idx = iy * n + ix;
                    if (u0 < 0 || v0 < 0 || u0 >= n - 1 || v0 >= n - 1)
                    {
                        _ix0[idx] = 0; _iy0[idx] = 0; _wx[idx] = 0; _wy[idx] = 0;
                    }
                    else
                    {
                        _ix0[idx] = u0; _iy0[idx] = v0; _wx[idx] = u - u0; _wy[idx] = v - v0;
                    }
                }
            }
        }

        public void Resample(double[] srcCentered, double[] dst)
        {
            int n = _n;
            for (int i = 0; i < n * n; i++)
            {
                int x0 = _ix0[i], y0 = _iy0[i];
                float wx = _wx[i], wy = _wy[i];
                int x1 = x0 + 1, y1 = y0 + 1;
                double a = srcCentered[y0 * n + x0], b = srcCentered[y0 * n + x1];
                double c = srcCentered[y1 * n + x0], d = srcCentered[y1 * n + x1];
                double top = a + (b - a) * wx;
                double bot = c + (d - c) * wx;
                dst[i] = top + (bot - top) * wy;
            }
        }
    }
}
