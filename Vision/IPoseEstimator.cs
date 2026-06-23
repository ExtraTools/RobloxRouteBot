using System.Numerics;

namespace RobloxRouteBot.Vision;

/// <summary>Межкадровая дельта позы «подобия вокруг центра»: поворот + сдвиг (масштаб игнорируем).</summary>
public readonly struct PoseDelta
{
    public float DTheta { get; init; }   // межкадровый поворот, радианы
    public Vector2 Shift { get; init; }  // остаточный сдвиг после де-ротации, px кадра
    public float Conf { get; init; }     // уверенность 0..1

    public static PoseDelta Identity => new() { DTheta = 0, Shift = Vector2.Zero, Conf = 0 };
}

/// <summary>
/// Оценщик ПОЗЫ (поворот+сдвиг) между кадрами. Отделён от IMotionEstimator, чтобы трансляционный путь
/// и калибровка остались нетронутыми. Stateful: держит предыдущий кадр/спектры.
/// </summary>
public interface IPoseEstimator
{
    string Name { get; }
    PoseDelta Submit(GrayFrame cur);
    void Reset();
}
