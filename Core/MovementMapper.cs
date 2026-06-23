using System.Numerics;
using RobloxRouteBot.Input;

namespace RobloxRouteBot.Core;

/// <summary>
/// Переводит желаемое направление движения в набор WASD при ЗАФИКСИРОВАННОЙ камере.
/// Камера не крутится => нет поворотной ошибки (главного источника дрейфа). Направление
/// аппроксимируется 8 сторонами (W/A/S/D и диагонали). Произвольные углы — будущая модель
/// с быстрым чередованием соседних направлений (ШИМ по направлению).
///
/// Система координат канваса: +X вправо, +Y ВНИЗ. «Вверх» по канвасу (−Y) = вперёд (W),
/// т.е. предполагаем, что камера смотрит «вверх по нарисованной карте».
/// </summary>
public static class MovementMapper
{
    // sin(22.5°): порог, дающий чистое деление на 8 направлений.
    private const float Threshold = 0.3826834f;

    public static MoveKey Map(Vector2 dir)
    {
        if (dir.LengthSquared() < 1e-6f) return MoveKey.None;
        dir = Vector2.Normalize(dir);

        MoveKey keys = MoveKey.None;

        // Вперёд/назад: «вверх» по канвасу — это −Y.
        if (dir.Y < -Threshold) keys |= MoveKey.Forward;
        else if (dir.Y > Threshold) keys |= MoveKey.Back;

        // Влево/вправо.
        if (dir.X > Threshold) keys |= MoveKey.Right;
        else if (dir.X < -Threshold) keys |= MoveKey.Left;

        return keys;
    }
}
