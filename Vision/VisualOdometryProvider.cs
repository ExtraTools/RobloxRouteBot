using System.Numerics;

namespace RobloxRouteBot.Vision;

/// <summary>
/// Основной «глаз» бота: визуальная одометрия с ПОВОРОТОМ (Fourier-Mellin).
///  Layer 1 (каждый тик): поза (поворот dθ + сдвиг) → накапливаем heading и интегрируем позицию
///     В МИРОВОЙ системе (сдвиг поворачивается на heading), поэтому путь остаётся привязан к миру,
///     даже если повернуть камеру.
///  Layer 2: ре-локализация по ориентирам (поворото-устойчивая) → мягкая коррекция дрейфа.
/// «Финиш» — только когда ИЗМЕРЕННАЯ позиция реально дошла.
/// </summary>
public sealed class VisualOdometryProvider : IPositionProvider
{
    private const float MotionGate = 0.06f;
    private const float HarvestConf = 0.45f;
    private const float SnapGain = 0.35f;
    private const float MaxStepRad = 0.30f;     // отсечка дикого скачка поворота за тик (~17°)

    private readonly ICaptureSource _cap;
    private readonly IPoseEstimator _pose;
    private readonly FrameTransform _ft;
    private readonly LandmarkMap _landmarks;
    private readonly int _n;

    private Vector2 _pos;
    private float _conf;
    private float _fps;

    public VisualOdometryProvider(ICaptureSource capture, IPoseEstimator pose,
        FrameTransform frame, LandmarkMap landmarks, int analysisSize = 256)
    {
        _cap = capture;
        _pose = pose;
        _ft = frame;
        _landmarks = landmarks;
        _n = analysisSize;
    }

    public string Name => "Зрение (Fourier-Mellin одометрия)";
    public float Confidence => _conf;

    public bool UseLandmarks { get; set; } = true;

    // ===== Телеметрия =====
    public string CaptureMode { get; private set; } = "—";
    public float Response { get; private set; }
    public bool SnappedThisTick { get; private set; }
    public float DriftSinceSnap { get; private set; }
    public int DupSkips { get; private set; }
    public int LandmarkCount => _landmarks.Count;
    public float Fps => _fps;
    public float HeadingDeg => _ft.Heading * 180f / MathF.PI;

    // ===== Доступ для калибровки/маппинга =====
    public FrameTransform Frame => _ft;
    public IPoseEstimator PoseEstimator => _pose;
    public ICaptureSource Capture => _cap;
    public int AnalysisSize => _n;

    public void Start(Vector2 startWorldPos)
    {
        _pos = startWorldPos;
        _conf = 0;
        _pose.Reset();
        _landmarks.Clear();
        SnappedThisTick = false;
        DriftSinceSnap = 0;
        DupSkips = 0;
    }

    public void SetPosition(Vector2 worldPos)
    {
        _pos = worldPos;
        _landmarks.Clear();
        _pose.Reset();
    }

    public Vector2 Update(Vector2 commandedDir, float speed, double dt)
    {
        SnappedThisTick = false;
        if (dt > 1e-4) _fps = 0.85f * _fps + 0.15f * (float)(1.0 / dt);

        var gray = _cap.GetGray(_n, _n, out bool dup);
        CaptureMode = _cap.Mode;
        if (gray == null) { _conf *= 0.9f; Response = 0; return _pos; }
        if (dup) { DupSkips++; return _pos; }

        var pd = _pose.Submit(gray);
        Response = pd.Conf;

        if (pd.Conf > MotionGate)
        {
            if (MathF.Abs(pd.DTheta) < MaxStepRad) _ft.AddHeading(pd.DTheta); // накапливаем поворот, отсекая скачки
            _pos += _ft.RotateScreenShiftToWorld(pd.Shift);
            _conf = pd.Conf;
        }
        else _conf *= 0.92f;

        if (UseLandmarks)
        {
            float h = _ft.Heading;
            if (_landmarks.TryRelocalize(gray, _pos, _ft, h, out var posAbs, out var snapConf))
            {
                Vector2 before = _pos;
                _pos += SnapGain * snapConf * (posAbs - _pos);
                DriftSinceSnap = Vector2.Distance(posAbs, before);
                SnappedThisTick = true;
            }
            if (pd.Conf > HarvestConf) _landmarks.Harvest(gray, _pos, _ft, h);
        }

        return _pos;
    }

    public void Dispose() { }
}
