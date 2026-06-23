using System.Numerics;

namespace RobloxRouteBot.Vision;

/// <summary>
/// Переиспользуемое FFT-ядро + фазовая корреляция — ОДИН проверенный источник математики для
/// PhaseCorrelationEstimator (сдвиг) и FourierMellinEstimator (поворот через лог-полярность).
/// Держит буферы (hann, scratch) под фиксированный размер. Не thread-safe — на каждый поток свой.
/// </summary>
public sealed class FourierProcessor
{
    private int _n;
    private double[]? _hann;
    private Complex[]? _row, _cross;
    private double[]? _surface;

    public int Size => _n;

    public double[] Hann(int n) { Ensure(n); return _hann!; }

    public void Ensure(int n)
    {
        if (_n == n && _hann != null) return;
        _n = n;
        _hann = new double[n * n];
        for (int y = 0; y < n; y++)
        {
            double wy = 0.5 * (1 - Math.Cos(2 * Math.PI * y / (n - 1)));
            for (int x = 0; x < n; x++)
            {
                double wx = 0.5 * (1 - Math.Cos(2 * Math.PI * x / (n - 1)));
                _hann[y * n + x] = wx * wy;
            }
        }
        _row = new Complex[n];
        _cross = new Complex[n * n];
        _surface = new double[n * n];
    }

    /// <summary>Спектр серого кадра (опц. окно Ханна). Возвращает НОВЫЙ массив (можно кэшировать как prev).</summary>
    public Complex[] Spectrum(GrayFrame f, bool window = true)
    {
        int n = f.Width;
        Ensure(n);
        var buf = new Complex[n * n];
        if (window) { var h = _hann!; for (int i = 0; i < n * n; i++) buf[i] = new Complex(f.Pixels[i] * h[i], 0); }
        else { for (int i = 0; i < n * n; i++) buf[i] = new Complex(f.Pixels[i], 0); }
        Fft2D(buf, n, false);
        return buf;
    }

    /// <summary>Спектр вещественного массива n×n (для лог-полярного образа).</summary>
    public Complex[] SpectrumOfReal(double[] real, int n, bool window = false)
    {
        Ensure(n);
        var buf = new Complex[n * n];
        if (window) { var h = _hann!; for (int i = 0; i < n * n; i++) buf[i] = new Complex(real[i] * h[i], 0); }
        else { for (int i = 0; i < n * n; i++) buf[i] = new Complex(real[i], 0); }
        Fft2D(buf, n, false);
        return buf;
    }

    /// <summary>
    /// Кросс-power фазовая корреляция двух спектров. Возвращает (сдвиг в px, уверенность 0..1).
    /// shift — насколько содержимое curSpec сместилось относительно prevSpec (cur ≈ prev, сдвинутый на shift).
    /// </summary>
    public (Vector2 shift, float conf) Correlate(Complex[] curSpec, Complex[] prevSpec, int n)
    {
        Ensure(n);
        var R = _cross!;
        for (int i = 0; i < n * n; i++)
        {
            Complex c = curSpec[i] * Complex.Conjugate(prevSpec[i]);
            double mag = c.Magnitude;
            R[i] = mag > 1e-12 ? c / mag : Complex.Zero;
        }
        Fft2D(R, n, inverse: true);

        var surf = _surface!;
        double peak = double.MinValue;
        int pi = 0;
        for (int i = 0; i < n * n; i++) { double v = R[i].Real; surf[i] = v; if (v > peak) { peak = v; pi = i; } }
        int px = pi % n, py = pi / n;

        double second = 0;
        int excl = Math.Max(2, n / 32);
        for (int y = 0; y < n; y++)
        {
            int dy = Math.Min(Math.Abs(y - py), n - Math.Abs(y - py));
            for (int x = 0; x < n; x++)
            {
                int dx = Math.Min(Math.Abs(x - px), n - Math.Abs(x - px));
                if (dx <= excl && dy <= excl) continue;
                double v = surf[y * n + x];
                if (v > second) second = v;
            }
        }
        float conf = peak > 1e-9 ? (float)Math.Clamp(1.0 - second / peak, 0.0, 1.0) : 0f;

        double sx = px + Parabolic(surf[py * n + Wrap(px - 1, n)], peak, surf[py * n + Wrap(px + 1, n)]);
        double sy = py + Parabolic(surf[Wrap(py - 1, n) * n + px], peak, surf[Wrap(py + 1, n) * n + px]);
        double shx = sx > n / 2.0 ? sx - n : sx;
        double shy = sy > n / 2.0 ? sy - n : sy;
        return (new Vector2((float)shx, (float)shy), conf);
    }

    private static double Parabolic(double l, double c, double r)
    {
        double denom = l - 2 * c + r;
        if (Math.Abs(denom) < 1e-12) return 0;
        return Math.Clamp(0.5 * (l - r) / denom, -1.0, 1.0);
    }

    private static int Wrap(int i, int n) => (i % n + n) % n;

    public static bool IsPow2(int n) => n > 0 && (n & (n - 1)) == 0;

    public void Fft2D(Complex[] d, int n, bool inverse)
    {
        var row = _row!;
        for (int r = 0; r < n; r++) { Array.Copy(d, r * n, row, 0, n); Fft1D(row, inverse); Array.Copy(row, 0, d, r * n, n); }
        Transpose(d, n);
        for (int r = 0; r < n; r++) { Array.Copy(d, r * n, row, 0, n); Fft1D(row, inverse); Array.Copy(row, 0, d, r * n, n); }
        Transpose(d, n);
        if (inverse) { double inv = 1.0 / ((double)n * n); for (int i = 0; i < n * n; i++) d[i] *= inv; }
    }

    private static void Transpose(Complex[] d, int n)
    {
        for (int y = 0; y < n; y++)
            for (int x = y + 1; x < n; x++)
                (d[y * n + x], d[x * n + y]) = (d[x * n + y], d[y * n + x]);
    }

    private static void Fft1D(Complex[] a, bool inverse)
    {
        int n = a.Length;
        for (int i = 1, j = 0; i < n; i++)
        {
            int bit = n >> 1;
            for (; (j & bit) != 0; bit >>= 1) j ^= bit;
            j ^= bit;
            if (i < j) (a[i], a[j]) = (a[j], a[i]);
        }
        for (int len = 2; len <= n; len <<= 1)
        {
            double ang = 2 * Math.PI / len * (inverse ? 1 : -1);
            Complex wlen = new(Math.Cos(ang), Math.Sin(ang));
            for (int i = 0; i < n; i += len)
            {
                Complex w = Complex.One;
                int half = len >> 1;
                for (int k = 0; k < half; k++)
                {
                    Complex u = a[i + k];
                    Complex v = a[i + k + half] * w;
                    a[i + k] = u + v;
                    a[i + k + half] = u - v;
                    w *= wlen;
                }
            }
        }
    }
}
