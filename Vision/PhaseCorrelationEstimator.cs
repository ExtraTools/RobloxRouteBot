using System.Numerics;

namespace RobloxRouteBot.Vision;

/// <summary>
/// Оценщик сдвига кадра через глобальную фазовую корреляцию (только трансляция). Тонкая обёртка над
/// FourierProcessor: кэширует спектр предыдущего кадра, отдаёт сдвиг текущего относительно него.
/// Идеально для top-down БЕЗ поворота камеры. Поворот ловит FourierMellinEstimator поверх того же ядра.
/// </summary>
public sealed class PhaseCorrelationEstimator : IMotionEstimator
{
    private readonly FourierProcessor _fp = new();
    private Complex[]? _prevSpec;
    private int _n;

    public string Name => "Фазовая корреляция (FFT)";

    public void Reset() => _prevSpec = null;

    public (Vector2 shift, float conf) Submit(GrayFrame cur)
    {
        int n = cur.Width;
        if (n != cur.Height || !FourierProcessor.IsPow2(n)) return (Vector2.Zero, 0f);

        var spec = _fp.Spectrum(cur);
        if (_prevSpec == null || _n != n) { _prevSpec = spec; _n = n; return (Vector2.Zero, 0f); }

        var res = _fp.Correlate(spec, _prevSpec, n);
        _prevSpec = spec;
        return res;
    }
}
