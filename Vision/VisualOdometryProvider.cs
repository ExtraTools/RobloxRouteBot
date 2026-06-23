using System.Numerics;

namespace RobloxRouteBot.Vision;

/// <summary>
/// Основной «глаз» бота: визуальная одометрия в two-layer.
///  Layer 1 (каждый тик): глобальный сдвиг кадра (фазовая корреляция) → интеграция позиции.
///  Layer 2 (по возможности): ре-локализация по запомненным ориентирам → мягкая коррекция дрейфа.
/// Всё в единой системе координат канваса через FrameTransform. Позиция = реальная (измеренная),
/// поэтому «финиш» наступает, только когда персонаж ДЕЙСТВИТЕЛЬНО дошёл, а не когда фантом доехал.
/// </summary>
public sealed class VisualOdometryProvider : IPositionProvider
{
    private const float MotionGate = 0.06f;   // ниже — кадр неинформативен, позицию держим
    private const float HarvestConf = 0.45f;  // выше — можно засевать новые ориентиры
    private const float SnapGain = 0.35f;     // сила притяжения к абсолютной позиции

    private readonly ICaptureSource _cap;
    private readonly IMotionEstimator _estimator;
    private readonly FrameTransform _ft;
    private readonly LandmarkMap _landmarks;
    private readonly int _n;

    private Vector2 _pos;
    private float _conf;
    private float _fps;

    public VisualOdometryProvider(ICaptureSource capture, IMotionEstimator estimator,
        FrameTransform frame, LandmarkMap landmarks, int analysisSize = 256)
    {
        _cap = capture;
        _estimator = estimator;
        _ft = frame;
        _landmarks = landmarks;
        _n = analysisSize;
    }

    public string Name => "Зрение (визуальная одометрия)";
    public float Confidence => _conf;

    public bool UseLandmarks { get; set; } = true;

    // ===== Телеметрия для UI =====
    public string CaptureMode { get; private set; } = "—";
    public float Response { get; private set; }
    public bool SnappedThisTick { get; private set; }
    public float DriftSinceSnap { get; private set; }
    public int DupSkips { get; private set; }
    public int LandmarkCount => _landmarks.Count;
    public float Fps => _fps;
    public Vector2 LastShiftPixels { get; private set; }

    // ===== Доступ для калибровки/маппинга =====
    public FrameTransform Frame => _ft;
    public IMotionEstimator Estimator => _estimator;
    public ICaptureSource Capture => _cap;
    public int AnalysisSize => _n;

    public void Start(Vector2 startWorldPos)
    {
        _pos = startWorldPos;
        _conf = 0;
        _estimator.Reset();
        _landmarks.Clear();
        SnappedThisTick = false;
        DriftSinceSnap = 0;
        DupSkips = 0;
        LastShiftPixels = Vector2.Zero;
    }

    public void SetPosition(Vector2 worldPos)
    {
        _pos = worldPos;
        _landmarks.Clear();   // ре-якорь вручную: старые ориентиры привязаны к дрейфовавшей траектории
        _estimator.Reset();
    }

    public Vector2 Update(Vector2 commandedDir, float speed, double dt)
    {
        SnappedThisTick = false;
        if (dt > 1e-4) _fps = 0.85f * _fps + 0.15f * (float)(1.0 / dt);

        var gray = _cap.GetGray(_n, _n, out bool dup);
        CaptureMode = _cap.Mode;
        if (gray == null) { _conf *= 0.9f; Response = 0; return _pos; }
        if (dup) { DupSkips++; return _pos; } // повтор кадра = нулевое движение, не интегрируем

        var (shift, mconf) = _estimator.Submit(gray);
        Response = mconf;
        LastShiftPixels = shift;

        if (mconf > MotionGate)
        {
            _pos += _ft.MapShiftToWorld(shift);
            _conf = mconf;
        }
        else
        {
            _conf *= 0.92f; // держим позицию, не интегрируем мусор
        }

        if (UseLandmarks)
        {
            if (_landmarks.TryRelocalize(gray, _pos, _ft, out var posAbs, out var snapConf))
            {
                Vector2 before = _pos;
                _pos += SnapGain * snapConf * (posAbs - _pos);
                DriftSinceSnap = Vector2.Distance(posAbs, before);
                SnappedThisTick = true;
            }
            if (mconf > HarvestConf) _landmarks.Harvest(gray, _pos, _ft);
        }

        return _pos;
    }

    public void Dispose() { }
}
