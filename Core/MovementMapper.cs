using System.Numerics;
using RobloxRouteBot.Input;
using RobloxRouteBot.Vision;

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

    /// <summary>
    /// Маппинг через откалиброванный базис: желаемое направление в канвасе раскладываем на
    /// (вперёд, вбок) = B⁻¹·dir, где столбцы B — направления движения при W и D. Это снимает
    /// предположение «вверх = вперёд»: при любой ориентации камеры жмём правильные клавиши.
    /// </summary>
    public static MoveKey Map(Vector2 dir, FrameTransform frame)
    {
        if (dir.LengthSquared() < 1e-6f) return MoveKey.None;
        var (forward, strafe) = frame.WorldDirToBodyCoeffs(Vector2.Normalize(dir));
        var body = new Vector2(forward, strafe);
        if (body.LengthSquared() < 1e-9f) return MoveKey.None;
        body = Vector2.Normalize(body);

        MoveKey keys = MoveKey.None;
        if (body.X > Threshold) keys |= MoveKey.Forward;
        else if (body.X < -Threshold) keys |= MoveKey.Back;
        if (body.Y > Threshold) keys |= MoveKey.Right;
        else if (body.Y < -Threshold) keys |= MoveKey.Left;
        return keys;
    }
}
