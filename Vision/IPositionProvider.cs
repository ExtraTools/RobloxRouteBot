using System.Numerics;

namespace RobloxRouteBot.Vision;

/// <summary>
/// «Глаза» бота: как он оценивает текущую позицию персонажа.
/// Ядро (BotEngine) от конкретной реализации не зависит — это и есть сменный модуль.
///   - DeadReckoningProvider — без обратной связи (open-loop), для проверки пайплайна.
///   - OpticalFlowProvider   — измеряет реальное движение по экрану (closed-loop, игра-агностично).
///   - (TODO) MinimapProvider / CoordOcrProvider — абсолютная позиция под конкретную игру.
/// Все позиции — в «мировых» единицах, совпадающих с координатами канваса маршрута.
/// </summary>
public interface IPositionProvider : IDisposable
{
    string Name { get; }

    /// <summary>Насколько можно доверять последней оценке (0..1).</summary>
    float Confidence { get; }

    /// <summary>Инициализация в известной стартовой точке маршрута.</summary>
    void Start(Vector2 startWorldPos);

    /// <summary>
    /// Один тик. commandedDir — нормализованное направление, которое контроллер сейчас командует
    /// (open-loop провайдер по нему и «едет»; провайдеры с реальным зрением его игнорируют).
    /// speed — скорость в мировых единицах/сек (для open-loop). dt — секунды с прошлого тика.
    /// Возвращает текущую оценку позиции.
    /// </summary>
    Vector2 Update(Vector2 commandedDir, float speed, double dt);
}
