using System.Numerics;

namespace RobloxRouteBot.Vision;

/// <summary>
/// Фолбэк-оценщик: блок-матчинг центрального патча по минимуму SAD (как было в OpticalFlowProvider,
/// вынесено в отдельный stateful-модуль). Проще и без зависимостей, но чувствителен к перекрытию
/// патча движущимися объектами и обрывается на границе окна поиска. Сабпиксель — параболой по SAD.
/// </summary>
public sealed class SadBlockEstimator : IMotionEstimator
{
    public int PatchSize { get; set; } = 96;
    public int SearchRadius { get; set; } = 24;
    public float MatchThreshold { get; set; } = 32f;

    public string Name => "Блок-матчинг (SAD)";

    private GrayFrame? _prev;

    public void Reset() => _prev = null;

    public (Vector2 shift, float conf) Submit(GrayFrame cur)
    {
        if (_prev == null || _prev.Width != cur.Width || _prev.Height != cur.Height)
        {
            _prev = cur;
            return (Vector2.Zero, 0f);
        }
        var res = Estimate(_prev, cur);
        _prev = cur;
        return res;
    }

    private (Vector2 shift, float conf) Estimate(GrayFrame prev, GrayFrame cur)
    {
        int W = prev.Width, H = prev.Height;
        int P = Math.Min(PatchSize, Math.Min(W, H) - 2 * SearchRadius - 2);
        if (P < 8) return (Vector2.Zero, 0f);
        int S = SearchRadius;
        int px = (W - P) / 2, py = (H - P) / 2;

        byte[] a = prev.Pixels, b = cur.Pixels;
        long best = long.MaxValue, secondBest = long.MaxValue;
        int bestDx = 0, bestDy = 0;
        var sadAt = new long[(2 * S + 1) * (2 * S + 1)];

        for (int dy = -S; dy <= S; dy++)
        {
            int sy = py + dy;
            if (sy < 0 || sy + P > H) { continue; }
            for (int dx = -S; dx <= S; dx++)
            {
                int sx = px + dx;
                if (sx < 0 || sx + P > W) continue;

                long sad = 0;
                for (int j = 0; j < P; j++)
                {
                    int aRow = (py + j) * W + px;
                    int bRow = (sy + j) * W + sx;
                    for (int i = 0; i < P; i++)
                    {
                        int d = a[aRow + i] - b[bRow + i];
                        sad += d >= 0 ? d : -d;
                    }
                    if (sad >= best) break;
                }
                sadAt[(dy + S) * (2 * S + 1) + (dx + S)] = sad;
                if (sad < best) { secondBest = best; best = sad; bestDx = dx; bestDy = dy; }
                else if (sad < secondBest) secondBest = sad;
            }
        }

        if (best == long.MaxValue) return (Vector2.Zero, 0f);

        float avgDiff = (float)best / (P * P);
        float conf = 1f - Math.Clamp(avgDiff / MatchThreshold, 0f, 1f);
        if (Math.Abs(bestDx) >= S || Math.Abs(bestDy) >= S) conf *= 0.3f;

        // Сабпиксель по соседним SAD вдоль каждой оси.
        float subDx = bestDx + SubPeak(SadSafe(sadAt, bestDx - 1, bestDy, S), best, SadSafe(sadAt, bestDx + 1, bestDy, S));
        float subDy = bestDy + SubPeak(SadSafe(sadAt, bestDx, bestDy - 1, S), best, SadSafe(sadAt, bestDx, bestDy + 1, S));

        return (new Vector2(subDx, subDy), conf);
    }

    private static long SadSafe(long[] arr, int dx, int dy, int S)
    {
        if (dx < -S || dx > S || dy < -S || dy > S) return long.MaxValue;
        long v = arr[(dy + S) * (2 * S + 1) + (dx + S)];
        return v == 0 ? long.MaxValue : v;
    }

    private static float SubPeak(long l, long c, long r)
    {
        if (l == long.MaxValue || r == long.MaxValue) return 0f;
        double denom = l - 2.0 * c + r;
        if (Math.Abs(denom) < 1e-6) return 0f;
        double d = 0.5 * (l - r) / denom;
        return (float)Math.Clamp(d, -1.0, 1.0);
    }
}
