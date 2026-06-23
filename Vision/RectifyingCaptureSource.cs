namespace RobloxRouteBot.Vision;

/// <summary>
/// Декоратор ICaptureSource: применяет IPM-ректификацию к серому кадру на ГРАНИЦЕ захвата, поэтому
/// ВСЕ потребители (FM, WasdCalibrator, LandmarkMap) видят один и тот же ректифицированный кадр —
/// иначе базис W/D калибровался бы в косом пространстве, а трекинг шёл в выпрямленном (рассинхрон).
/// Лайв-превью (TryGetBgra) отдаёт СЫРОЙ кадр — пользователь видит настоящую игру, а не варп.
/// tilt=0 → pass-through, ноль накладных.
/// </summary>
public sealed class RectifyingCaptureSource : ICaptureSource
{
    private readonly ICaptureSource _inner;
    private readonly InversePerspectiveMap _ipm = new();
    private GrayFrame? _scratch;

    private float _tilt;
    private float _fov = 70f * MathF.PI / 180f;
    private int _builtN;

    public RectifyingCaptureSource(ICaptureSource inner) => _inner = inner;

    public string Mode => _inner.Mode;

    /// <summary>Угол наклона камеры (рад, 0 = строго сверху) и вертикальный FOV (рад).</summary>
    public void SetTilt(float tiltRad, float fovRad)
    {
        _tilt = tiltRad;
        _fov = fovRad > 0.01f ? fovRad : _fov;
        _builtN = 0; // форсируем пересборку LUT
    }

    public bool TryGetBgra(out byte[] bgra, out int width, out int height)
        => _inner.TryGetBgra(out bgra, out width, out height); // превью — всегда сырой кадр

    public GrayFrame? GetGray(int targetW, int targetH, out bool isDuplicate)
    {
        var raw = _inner.GetGray(targetW, targetH, out isDuplicate);
        if (raw == null) return null;
        if (MathF.Abs(_tilt) < 1e-4f) return raw; // identity — текущее поведение

        if (_builtN != raw.Width) { _ipm.Build(raw.Width, _fov, _tilt); _builtN = raw.Width; }
        if (_scratch == null || _scratch.Width != raw.Width || _scratch.Height != raw.Height)
            _scratch = new GrayFrame(raw.Width, raw.Height);
        _ipm.Rectify(raw, _scratch);
        return _scratch;
    }
}
