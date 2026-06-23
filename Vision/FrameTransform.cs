using System.Numerics;

namespace RobloxRouteBot.Vision;

/// <summary>
/// Единственный источник истины по системам координат. Заменяет россыпь флагов
/// (InvertX/InvertY/SwapXy/RotationDeg) одним откалиброванным базисом.
///
/// Все рабочие координаты — в пикселях канваса (там же, где нарисован маршрут):
/// +X вправо, +Y вниз. «Зрение» меряет сдвиг картинки в пикселях кадра анализа (256×256).
///
/// Две задачи:
///  1. MapShiftToWorld(сдвиг кадра) → смещение персонажа в канвасе. Персонаж едет ПРОТИВ
///     движения картинки (follow-камера), поэтому знак минус. Масштаб Scale — канвас-px на px анализа.
///  2. WorldDirToBodyCoeffs(желаемое направление) → (вперёд, вбок) через B⁻¹, где столбцы B —
///     канвас-направления, в которые реально едет перс при нажатии W и D (из калибровки).
///     Это убирает хардкод «вверх = вперёд»: работает при любом повороте камеры.
/// </summary>
public sealed class FrameTransform
{
    private readonly object _gate = new();

    // Столбцы базиса B: канвас-направления движения при W и D (единичные).
    private Vector2 _fwd = new(0f, -1f); // по умолчанию: W = вверх по канвасу
    private Vector2 _str = new(1f, 0f);  // по умолчанию: D = вправо

    public float ScaleX { get; set; } = 1f;
    public float ScaleY { get; set; } = 1f;

    /// <summary>Аварийный тумблер знака интеграции позиции (если конвенция сдвига зеркальна).</summary>
    public bool InvertMotion { get; set; }

    public bool Calibrated { get; private set; }

    public Vector2 Forward { get { lock (_gate) return _fwd; } }
    public Vector2 Strafe  { get { lock (_gate) return _str; } }

    /// <summary>Установить базис из калибровки. Векторы — канвас-направления при W/D (любой длины).</summary>
    public void SetBasis(Vector2 forwardCanvas, Vector2 strafeCanvas)
    {
        if (forwardCanvas.LengthSquared() < 1e-6f || strafeCanvas.LengthSquared() < 1e-6f) return;
        var f = Vector2.Normalize(forwardCanvas);
        var s = Vector2.Normalize(strafeCanvas);
        // Вырожденный базис (W и D почти коллинеарны) — не принимаем.
        float det = f.X * s.Y - s.X * f.Y;
        if (MathF.Abs(det) < 0.15f) return;
        lock (_gate) { _fwd = f; _str = s; Calibrated = true; }
    }

    public void ResetBasis()
    {
        lock (_gate) { _fwd = new Vector2(0f, -1f); _str = new Vector2(1f, 0f); Calibrated = false; }
    }

    /// <summary>Сдвиг картинки (px анализа) → смещение персонажа в канвасе (perс едет против картинки).</summary>
    public Vector2 MapShiftToWorld(Vector2 shiftPx)
    {
        float sgn = InvertMotion ? 1f : -1f;
        return new Vector2(sgn * shiftPx.X * ScaleX, sgn * shiftPx.Y * ScaleY);
    }

    /// <summary>Желаемое направление в канвасе → коэффициенты (вперёд, вбок) = B⁻¹·dir.</summary>
    public (float forward, float strafe) WorldDirToBodyCoeffs(Vector2 dir)
    {
        Vector2 f, s;
        lock (_gate) { f = _fwd; s = _str; }
        float det = f.X * s.Y - s.X * f.Y;
        if (MathF.Abs(det) < 1e-6f) return (0f, 0f);
        float inv = 1f / det;
        // B⁻¹ = 1/det * [[ s.Y, -s.X], [-f.Y, f.X]]
        float forward = (s.Y * dir.X - s.X * dir.Y) * inv;
        float strafe  = (-f.Y * dir.X + f.X * dir.Y) * inv;
        return (forward, strafe);
    }

    /// <summary>Экранный офсет (px анализа от центра) → офсет в канвасе. Для ориентиров.</summary>
    public Vector2 ScreenOffsetToWorld(Vector2 offsetPx)
        => new(offsetPx.X * ScaleX, offsetPx.Y * ScaleY);

    /// <summary>Офсет в канвасе → экранный офсет (px анализа). Обратное к ScreenOffsetToWorld.</summary>
    public Vector2 WorldOffsetToScreen(Vector2 offsetWorld)
        => new(ScaleX > 1e-6f ? offsetWorld.X / ScaleX : 0f,
               ScaleY > 1e-6f ? offsetWorld.Y / ScaleY : 0f);
}
