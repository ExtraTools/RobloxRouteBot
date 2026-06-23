using System.Numerics;

namespace RobloxRouteBot.Vision;

/// <summary>
/// Оценщик ПОЗЫ (поворот + сдвиг) по Реди-Чаттерджи (Fourier-Mellin), чистый managed.
///  1) |FFT| текущего кадра инвариантна к сдвигу. Центрируем, давим низкие частоты high-pass маской,
///     ресэмплим в лог-полярность → фазовая корреляция по θ-оси даёт МЕЖКАДРОВЫЙ ПОВОРОТ dθ.
///  2) Де-ротируем кадр на −dθ и обычной фазовой корреляцией берём остаточный СДВИГ. Снимаем
///     неоднозначность 180° перебором {dθ, dθ+π} по уверенности трансляции.
/// Это и есть устойчивость к повороту камеры: путь остаётся привязан к миру (поза копится в провайдере).
/// </summary>
public sealed class FourierMellinEstimator : IPoseEstimator
{
    private const float RotGate = 0.18f;        // ниже — повороту не верим, считаем 0
    private const float SmallAngle = 0.026f;    // ~1.5°: меньше — поворот игнорируем (дёшево)

    private readonly FourierProcessor _fp = new();
    private int _n;
    private ImageWarp.LogPolar? _lp;
    private double[]? _mag, _logpolar, _highpass;

    private GrayFrame? _prevGray;
    private Complex[]? _prevSpatialSpec;
    private Complex[]? _prevLogSpec;

    public string Name => "Fourier-Mellin (поворот+сдвиг)";

    public void Reset()
    {
        _prevGray = null;
        _prevSpatialSpec = null;
        _prevLogSpec = null;
    }

    public PoseDelta Submit(GrayFrame cur)
    {
        int n = cur.Width;
        if (n != cur.Height || !FourierProcessor.IsPow2(n)) return PoseDelta.Identity;
        Ensure(n);

        var specCur = _fp.Spectrum(cur, window: true);   // спектр кадра (для |F| И как prev для трансляции)
        BuildCenteredMag(specCur, n);
        _lp!.Resample(_mag!, _logpolar!);
        var logSpec = _fp.SpectrumOfReal(_logpolar!, n);

        if (_prevGray == null)
        {
            _prevGray = cur; _prevSpatialSpec = specCur; _prevLogSpec = logSpec;
            return PoseDelta.Identity;
        }

        var (lpShift, rotConf) = _fp.Correlate(logSpec, _prevLogSpec!, n);
        // θ-ось лог-полярности идёт по [0,π) → угол на бин = π/n.
        double dTheta = NormAngle(lpShift.X * (Math.PI / n));

        PoseDelta result;
        if (rotConf < RotGate || Math.Abs(dTheta) < SmallAngle)
        {
            // Поворот незначим/ненадёжен → только трансляция (дёшево, переиспользуем specCur).
            var (shift, tconf) = _fp.Correlate(specCur, _prevSpatialSpec!, n);
            result = new PoseDelta { DTheta = 0f, Shift = shift, Conf = tconf };
        }
        else
        {
            float bestConf = -1f, bestTheta = 0f;
            Vector2 bestShift = Vector2.Zero;
            foreach (double cand in new[] { dTheta, NormAngle(dTheta + Math.PI) })
            {
                var deRot = ImageWarp.RotateAboutCenter(cur, (float)(-cand));
                var specDeRot = _fp.Spectrum(deRot, window: true);
                var (shift, tconf) = _fp.Correlate(specDeRot, _prevSpatialSpec!, n);
                if (tconf > bestConf) { bestConf = tconf; bestTheta = (float)cand; bestShift = shift; }
            }
            result = new PoseDelta { DTheta = bestTheta, Shift = bestShift, Conf = Math.Min(rotConf, bestConf) };
        }

        _prevGray = cur; _prevSpatialSpec = specCur; _prevLogSpec = logSpec;
        return result;
    }

    private void Ensure(int n)
    {
        if (_n == n && _lp != null) return;
        _n = n;
        _lp = new ImageWarp.LogPolar(n);
        _mag = new double[n * n];
        _logpolar = new double[n * n];
        _highpass = new double[n * n];
        for (int y = 0; y < n; y++)
        {
            float fy = (y - n / 2f) / n;
            for (int x = 0; x < n; x++)
            {
                float fx = (x - n / 2f) / n;
                // High-pass Реди: давит DC/низкие частоты, поднимает середину.
                double X = Math.Cos(Math.PI * fx) * Math.Cos(Math.PI * fy);
                _highpass[y * n + x] = (1.0 - X) * (2.0 - X);
            }
        }
        Reset();
    }

    /// <summary>|FFT| с DC в центре (fftshift) × high-pass. В _mag.</summary>
    private void BuildCenteredMag(Complex[] spec, int n)
    {
        var mag = _mag!;
        var hp = _highpass!;
        int half = n / 2;
        for (int y = 0; y < n; y++)
        {
            int sy = (y + half) % n; // fftshift
            for (int x = 0; x < n; x++)
            {
                int sx = (x + half) % n;
                double m = spec[sy * n + sx].Magnitude;
                mag[y * n + x] = Math.Log(1.0 + m) * hp[y * n + x];
            }
        }
    }

    private static double NormAngle(double a)
    {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
