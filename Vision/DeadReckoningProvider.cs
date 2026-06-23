using System.Numerics;

namespace RobloxRouteBot.Vision;

/// <summary>
/// Open-loop «глаза»: обратной связи нет, позиция предполагается равной интегралу команд.
/// В симуляции дот идёт по маршруту идеально; в реальной игре персонаж будет дрейфовать,
/// потому что этот провайдер НЕ видит, куда он реально пришёл. Базовая линия для сравнения.
/// </summary>
public sealed class DeadReckoningProvider : IPositionProvider
{
    private Vector2 _pos;

    public string Name => "Open-loop (без обратной связи)";
    public float Confidence => 1f; // формально «уверен», но это слепая вера в команды

    public void Start(Vector2 startWorldPos) => _pos = startWorldPos;

    public Vector2 Update(Vector2 commandedDir, float speed, double dt)
    {
        _pos += commandedDir * (float)(speed * dt);
        return _pos;
    }

    public void Dispose() { }
}
