using System.Numerics;

namespace RobloxRouteBot.Vision;

public sealed class OpticalFlowSettings
{
    public int CaptureWidth { get; set; } = 320;
    public int CaptureHeight { get; set; } = 180;
    public int PatchSize { get; set; } = 64;
    public int SearchRadius { get; set; } = 18;

    /// <summary>Сколько мировых единиц приходится на 1 пиксель сдвига кадра (калибруется).</summary>
    public float WorldPerPixel { get; set; } = 1.0f;

    public bool InvertX { get; set; } = false;
    public bool InvertY { get; set; } = false;
    public bool SwapXy { get; set; } = false;

    /// <summary>Поворот вектора потока в систему координат канваса, градусы.</summary>
    public float RotationDeg { get; set; } = 0f;

    /// <summary>Порог среднего модуля разности (0..255), выше которого матч считается ненадёжным.</summary>
    public float MatchThreshold { get; set; } = 32f;
}

/// <summary>
/// Closed-loop «глаза»: оценивает реальное смещение персонажа, измеряя, как «уезжает» картинка
/// под ним. Блок-матчинг: берём центральный патч прошлого кадра и ищем его в текущем кадре в
/// окне ±SearchRadius по минимуму SAD. Найденный сдвиг (dx,dy) — это движение мира на экране;
/// движение персонажа в мире ≈ обратное ему, с калибровкой масштаба/осей.
///
/// Поток относительный: ошибка медленно интегрируется, поэтому периодически нужен абсолютный
/// «якорь» (ориентир/упор в стену) для обнуления — это следующий слой поверх.
/// </summary>
public sealed class OpticalFlowProvider : IPositionProvider
{
    private readonly Func<IntPtr> _hwndSource;
    private readonly ScreenCapture _capture = new();
    private readonly OpticalFlowSettings _s;

    private GrayFrame? _prev;
    private Vector2 _pos;
    private float _confidence;

    public OpticalFlowProvider(Func<IntPtr> hwndSource, OpticalFlowSettings settings)
    {
        _hwndSource = hwndSource;
        _s = settings;
    }

    public string Name => "Оптический поток (closed-loop, универсально)";
    public float Confidence => _confidence;

    /// <summary>Последний измеренный сдвиг кадра в пикселях — для отладки/калибровки в UI.</summary>
    public Vector2 LastShiftPixels { get; private set; }

    /// <summary>Настройки (общий объект с UI калибровки — правки применяются на лету).</summary>
    public OpticalFlowSettings Settings => _s;

    public void Start(Vector2 startWorldPos)
    {
        _pos = startWorldPos;
        _prev = null;
        _confidence = 0;
        LastShiftPixels = Vector2.Zero;
    }

    public void SetPosition(Vector2 worldPos) => _pos = worldPos;

    public Vector2 Update(Vector2 commandedDir, float speed, double dt)
    {
        var (frame, shift, conf) = MeasureStep();
        if (frame == null || conf <= 0f) return _pos; // нет кадра / ненадёжно — позицию не двигаем
        _pos += MapShiftToWorld(shift);
        return _pos;
    }

    /// <summary>
    /// Один шаг измерения без интеграции позиции: захват кадра и оценка сдвига к предыдущему.
    /// Используется и движком, и панелью калибровки. frame == null => окно недоступно.
    /// </summary>
    public (GrayFrame? frame, Vector2 shift, float conf) MeasureStep()
    {
        IntPtr hwnd = _hwndSource();
        var frame = _capture.Capture(hwnd, _s.CaptureWidth, _s.CaptureHeight);
        if (frame == null)
        {
            _confidence = 0;
            return (null, Vector2.Zero, 0f);
        }

        if (_prev == null || _prev.Width != frame.Width || _prev.Height != frame.Height)
        {
            _prev = frame;
            _confidence = 0;
            LastShiftPixels = Vector2.Zero;
            return (frame, Vector2.Zero, 0f);
        }

        var (shift, conf) = EstimateShift(_prev, frame);
        _prev = frame;
        _confidence = conf;
        LastShiftPixels = shift;
        return (frame, shift, conf);
    }

    /// <summary>Сдвиг картинки (px) -> смещение персонажа в мире, с калибровкой осей/масштаба.</summary>
    public Vector2 MapShiftToWorld(Vector2 shift)
    {
        float mx = -shift.X; // мир движется обратно картинке
        float my = -shift.Y;
        if (_s.SwapXy) (mx, my) = (my, mx);
        if (_s.InvertX) mx = -mx;
        if (_s.InvertY) my = -my;

        if (MathF.Abs(_s.RotationDeg) > 0.01f)
        {
            float a = _s.RotationDeg * MathF.PI / 180f;
            float c = MathF.Cos(a), s = MathF.Sin(a);
            (mx, my) = (mx * c - my * s, mx * s + my * c);
        }

        return new Vector2(mx, my) * _s.WorldPerPixel;
    }

    /// <summary>Блок-матчинг центрального патча. Возвращает (сдвиг в пикселях, уверенность 0..1).</summary>
    private (Vector2 shift, float conf) EstimateShift(GrayFrame prev, GrayFrame cur)
    {
        int W = prev.Width, H = prev.Height;
        int P = Math.Min(_s.PatchSize, Math.Min(W, H) - 2 * _s.SearchRadius - 2);
        if (P < 8) return (Vector2.Zero, 0f);
        int S = _s.SearchRadius;

        int px = (W - P) / 2;
        int py = (H - P) / 2;

        byte[] a = prev.Pixels, b = cur.Pixels;

        long best = long.MaxValue;
        int bestDx = 0, bestDy = 0;

        for (int dy = -S; dy <= S; dy++)
        {
            int sy = py + dy;
            if (sy < 0 || sy + P > H) continue;
            for (int dx = -S; dx <= S; dx++)
            {
                int sx = px + dx;
                if (sx < 0 || sx + P > W) continue;

                long sad = 0;
                for (int j = 0; j < P; j++)
                {
                    int aRow = (py + j) * W + px;
                    int bRow = (sy + j) * W + sx;
                    for (int i = 0; i < P; i++)
                    {
                        int d = a[aRow + i] - b[bRow + i];
                        sad += d >= 0 ? d : -d;
                    }
                    if (sad >= best) break; // ранний выход — кандидат уже хуже лучшего
                }

                if (sad < best)
                {
                    best = sad;
                    bestDx = dx;
                    bestDy = dy;
                }
            }
        }

        if (best == long.MaxValue) return (Vector2.Zero, 0f);

        float avgDiff = (float)best / (P * P);
        float conf = 1f - Math.Clamp(avgDiff / _s.MatchThreshold, 0f, 1f);

        // Матч у самой границы окна => реальный сдвиг мог превысить SearchRadius: режем доверие.
        if (Math.Abs(bestDx) >= S || Math.Abs(bestDy) >= S) conf *= 0.3f;

        return (new Vector2(bestDx, bestDy), conf);
    }

    public void Dispose() { }
}
