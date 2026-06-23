using System.Numerics;

namespace RobloxRouteBot.Core;

/// <summary>
/// Контроллер следования по маршруту. v1 — поиск по вэйпоинтам с упреждением (look-ahead):
/// целимся в точку маршрута, лежащую на расстоянии LookAhead впереди ближайшей к нам точки.
/// Это и есть «следит за траекторией в реальном времени»: на каждом тике пересчитывает курс
/// от ТЕКУЩЕЙ (измеренной) позиции — поэтому при наличии обратной связи дрейф не копится.
/// </summary>
public sealed class PurePursuitController
{
    private IReadOnlyList<Vector2> _route = Array.Empty<Vector2>();
    private int _searchFrom;

    /// <summary>Радиус «дошёл до финала». Должен перекрывать амплитуду 8-сторонней осцилляции.</summary>
    public float ArriveRadius { get; set; } = 18f;

    /// <summary>Дистанция упреждения цели вдоль маршрута.</summary>
    public float LookAhead { get; set; } = 28f;

    public Vector2 Start => _route.Count > 0 ? _route[0] : Vector2.Zero;

    public void SetRoute(IReadOnlyList<Vector2> route)
    {
        _route = route;
        _searchFrom = 0;
    }

    public void Reset() => _searchFrom = 0;

    /// <summary>
    /// Вернуть (направление, финиш). direction нормализовано в координатах канваса (+Y вниз).
    /// </summary>
    public (Vector2 dir, bool done) Compute(Vector2 pos)
    {
        int n = _route.Count;
        if (n < 2) return (Vector2.Zero, true);

        // Финиш — когда близко к последней точке.
        if (Vector2.Distance(pos, _route[n - 1]) <= ArriveRadius)
            return (Vector2.Zero, true);

        // Ближайшая точка маршрута (ищем вперёд от прошлой, чтобы не «возвращаться»).
        int nearest = _searchFrom;
        float bestDist = float.MaxValue;
        for (int i = _searchFrom; i < n; i++)
        {
            float d = Vector2.DistanceSquared(pos, _route[i]);
            if (d < bestDist)
            {
                bestDist = d;
                nearest = i;
            }
        }
        _searchFrom = nearest;

        // Финиш по ПРОЕКЦИИ: мы на последнем сегменте и спроецировались за последнюю точку.
        // Снимает зависимость финиша от точного попадания в маленький радиус при дискретном (8-сторонами)
        // управлении — иначе перс может вечно кружить рядом, не входя в ArriveRadius.
        if (nearest >= n - 2)
        {
            Vector2 seg = _route[n - 1] - _route[n - 2];
            if (seg.LengthSquared() > 1e-6f && Vector2.Dot(pos - _route[n - 1], seg) >= 0f)
                return (Vector2.Zero, true);
        }

        // Идём вперёд по маршруту, пока не накопим LookAhead.
        int target = nearest;
        float acc = 0f;
        for (int i = nearest; i < n - 1; i++)
        {
            acc += Vector2.Distance(_route[i], _route[i + 1]);
            target = i + 1;
            if (acc >= LookAhead) break;
        }

        Vector2 toTarget = _route[target] - pos;
        if (toTarget.LengthSquared() < 1e-6f) return (Vector2.Zero, false);
        return (Vector2.Normalize(toTarget), false);
    }
}
