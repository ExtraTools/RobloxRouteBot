using System.Numerics;

namespace RobloxRouteBot.Vision;

/// <summary>
/// Оценщик глобального сдвига кадра. Stateful: внутри держит предыдущий кадр, Submit отдаёт сдвиг
/// текущего относительно него. Реализации взаимозаменяемы (фазовая корреляция — основная, SAD — фолбэк).
/// </summary>
public interface IMotionEstimator
{
    string Name { get; }

    /// <summary>
    /// Подать кадр. Возвращает (сдвиг картинки в px, уверенность 0..1) относительно прошлого кадра.
    /// На самом первом кадре (нет предыдущего) — ((0,0), 0).
    /// shift — насколько СОДЕРЖИМОЕ кадра сместилось (cur ≈ prev, сдвинутый на shift).
    /// </summary>
    (Vector2 shift, float conf) Submit(GrayFrame cur);

    void Reset();
}
