using System.Numerics;

namespace RobloxRouteBot.Vision;

/// <summary>
/// Слой 2 (гашение дрейфа) — идея пользователя: бот ЗАПОМИНАЕТ контрастные пиксельные патчи вдоль
/// маршрута с их мировой позицией. Узнаёт снова (NCC вокруг предсказанного места) → подтягивает позицию.
/// Поворото-устойчиво: храним heading на момент запоминания; при матче поворачиваем шаблон на
/// относительный угол (thetaW − harvestHeading), а прогноз/реконструкция идут через повёрнутые
/// преобразования FrameTransform. Никакой постоянной БД — всё строится на лету и стирается на Start.
/// </summary>
public sealed class LandmarkMap
{
    private sealed class Landmark
    {
        public required float[] Template;
        public required int P;
        public required Vector2 WorldPos;
        public float TemplateMean;
        public float TemplateNorm;
        public float Score;
        public float HarvestHeading;
        public int Misses;
    }

    public int PatchSize { get; set; } = 24;
    public int SearchRadius { get; set; } = 20;
    public float NccMin { get; set; } = 0.5f;
    public float PeakMargin { get; set; } = 0.06f;
    public int GateRadiusPx { get; set; } = 18;
    public int MaxLandmarks { get; set; } = 64;
    public float HarvestVarMin { get; set; } = 14f;
    public int GridDivs { get; set; } = 4;
    public int MaxRelocPerTick { get; set; } = 8;
    public float DerotateThreshold { get; set; } = 0.05f; // ~3°: меньше — NCC терпит без поворота шаблона

    private readonly List<Landmark> _lm = new();
    private int _relocCursor;
    private float[]? _nccSurf;

    public int Count => _lm.Count;

    public void Clear() { _lm.Clear(); _relocCursor = 0; }

    public bool TryRelocalize(GrayFrame cur, Vector2 pos, FrameTransform ft, float thetaW,
                              out Vector2 posAbs, out float conf)
    {
        posAbs = pos; conf = 0f;
        if (_lm.Count == 0) return false;

        int n = cur.Width;
        float cx = n / 2f, cy = n / 2f;
        int P = PatchSize, R = SearchRadius;

        Vector2 acc = Vector2.Zero;
        float wsum = 0f;
        int processed = 0;

        for (int s = 0; s < _lm.Count && processed < MaxRelocPerTick; s++)
        {
            var lm = _lm[(_relocCursor + s) % _lm.Count];

            Vector2 predOff = ft.WorldOffsetToScreen(lm.WorldPos - pos, thetaW);
            float px = cx + predOff.X, py = cy + predOff.Y;
            if (px - P / 2f - R < 0 || px + P / 2f + R >= n || py - P / 2f - R < 0 || py + P / 2f + R >= n)
                continue;

            processed++;

            // Поворот шаблона под текущую ориентацию кадра (если камера успела повернуться).
            float relRot = thetaW - lm.HarvestHeading;
            float[] tpl; float tmean, tnorm;
            if (MathF.Abs(relRot) > DerotateThreshold)
                RotatePatch(lm.Template, lm.P, relRot, out tpl, out tmean, out tnorm);
            else { tpl = lm.Template; tmean = lm.TemplateMean; tnorm = lm.TemplateNorm; }

            int span = 2 * R + 1;
            if (_nccSurf == null || _nccSurf.Length != span * span) _nccSurf = new float[span * span];
            var surf = _nccSurf;
            int baseX = (int)MathF.Round(px - P / 2f), baseY = (int)MathF.Round(py - P / 2f);

            float best = -2f; int bi = 0;
            for (int dy = -R; dy <= R; dy++)
                for (int dx = -R; dx <= R; dx++)
                {
                    float ncc = Ncc(cur, baseX + dx, baseY + dy, tpl, tmean, tnorm, P);
                    int idx = (dy + R) * span + (dx + R);
                    surf[idx] = ncc;
                    if (ncc > best) { best = ncc; bi = idx; }
                }
            int bdx = bi % span - R, bdy = bi / span - R;

            int excl = Math.Max(2, R / 6);
            float second = -2f;
            for (int dy = -R; dy <= R; dy++)
                for (int dx = -R; dx <= R; dx++)
                {
                    if (Math.Abs(dx - bdx) <= excl && Math.Abs(dy - bdy) <= excl) continue;
                    float v = surf[(dy + R) * span + (dx + R)];
                    if (v > second) second = v;
                }

            if (best < NccMin || (best - second) < PeakMargin) { lm.Misses++; continue; }
            if (bdx * bdx + bdy * bdy > GateRadiusPx * GateRadiusPx) { lm.Misses++; continue; }

            lm.Misses = 0;
            Vector2 matchedOff = new(px + bdx - cx, py + bdy - cy);
            Vector2 cand = lm.WorldPos - ft.ScreenOffsetToWorld(matchedOff, thetaW);
            float w = best;
            acc += cand * w;
            wsum += w;
        }
        _relocCursor = (_relocCursor + processed) % Math.Max(1, _lm.Count);

        if (wsum <= 0f) return false;
        posAbs = acc / wsum;
        conf = Math.Clamp(wsum / Math.Min(_lm.Count, MaxRelocPerTick), 0f, 1f);
        return true;
    }

    public void Harvest(GrayFrame cur, Vector2 pos, FrameTransform ft, float thetaW)
    {
        int n = cur.Width;
        int P = PatchSize;
        float cx = n / 2f, cy = n / 2f;
        float minWorld = 26f * 0.5f * (ft.ScaleX + ft.ScaleY);
        int cell = n / GridDivs;
        float centralR = n * 0.18f;

        for (int gy = 0; gy < GridDivs; gy++)
        {
            for (int gx = 0; gx < GridDivs; gx++)
            {
                if (_lm.Count >= MaxLandmarks) break;
                int bx = gx * cell + cell / 2;
                int by = gy * cell + cell / 2;
                if (MathF.Abs(bx - cx) < centralR && MathF.Abs(by - cy) < centralR) continue;
                if (bx - P / 2 < 0 || bx + P / 2 >= n || by - P / 2 < 0 || by + P / 2 >= n) continue;

                ExtractStats(cur, bx - P / 2, by - P / 2, P, out float mean, out float norm, out float std);
                if (std < HarvestVarMin) continue;

                Vector2 world = pos + ft.ScreenOffsetToWorld(new Vector2(bx - cx, by - cy), thetaW);
                bool near = false;
                foreach (var e in _lm)
                    if (Vector2.DistanceSquared(e.WorldPos, world) < minWorld * minWorld) { near = true; break; }
                if (near) continue;

                _lm.Add(MakeLandmark(cur, bx - P / 2, by - P / 2, P, world, std, mean, norm, thetaW));
            }
        }

        if (_lm.Count > MaxLandmarks)
        {
            _lm.Sort((a, b) =>
            {
                int m = b.Misses.CompareTo(a.Misses);
                return m != 0 ? m : a.Score.CompareTo(b.Score);
            });
            _lm.RemoveRange(MaxLandmarks, _lm.Count - MaxLandmarks);
        }
    }

    private static Landmark MakeLandmark(GrayFrame f, int x0, int y0, int P, Vector2 world,
                                         float std, float mean, float norm, float heading)
    {
        var tpl = new float[P * P];
        for (int j = 0; j < P; j++)
            for (int i = 0; i < P; i++)
                tpl[j * P + i] = f.At(x0 + i, y0 + j);
        return new Landmark
        {
            Template = tpl, P = P, WorldPos = world, Score = std,
            TemplateMean = mean, TemplateNorm = norm, HarvestHeading = heading,
        };
    }

    private static void ExtractStats(GrayFrame f, int x0, int y0, int P, out float mean, out float norm, out float std)
    {
        double sum = 0, sum2 = 0; int cnt = P * P;
        for (int j = 0; j < P; j++)
            for (int i = 0; i < P; i++)
            {
                int v = f.At(x0 + i, y0 + j);
                sum += v; sum2 += (double)v * v;
            }
        mean = (float)(sum / cnt);
        double var = Math.Max(0, sum2 / cnt - (double)mean * mean);
        std = (float)Math.Sqrt(var);
        norm = (float)Math.Sqrt(Math.Max(1e-6, sum2 - sum * sum / cnt));
    }

    /// <summary>Повернуть шаблон вокруг центра на angle (билинейно, край = clamp), пересчитать mean/norm.</summary>
    private static void RotatePatch(float[] src, int P, float angle, out float[] dst, out float mean, out float norm)
    {
        dst = new float[P * P];
        float c = MathF.Cos(angle), s = MathF.Sin(angle);
        float cc = (P - 1) / 2f;
        double sum = 0, sum2 = 0;
        for (int oy = 0; oy < P; oy++)
            for (int ox = 0; ox < P; ox++)
            {
                float dx = ox - cc, dy = oy - cc;
                float sx = cc + c * dx + s * dy;
                float sy = cc - s * dx + c * dy;
                float v = SampleClamp(src, P, sx, sy);
                dst[oy * P + ox] = v;
                sum += v; sum2 += (double)v * v;
            }
        int cnt = P * P;
        mean = (float)(sum / cnt);
        norm = (float)Math.Sqrt(Math.Max(1e-6, sum2 - sum * sum / cnt));
    }

    private static float SampleClamp(float[] px, int P, float x, float y)
    {
        x = Math.Clamp(x, 0, P - 1); y = Math.Clamp(y, 0, P - 1);
        int x0 = (int)x, y0 = (int)y;
        int x1 = Math.Min(x0 + 1, P - 1), y1 = Math.Min(y0 + 1, P - 1);
        float fx = x - x0, fy = y - y0;
        float a = px[y0 * P + x0], b = px[y0 * P + x1], c = px[y1 * P + x0], d = px[y1 * P + x1];
        float top = a + (b - a) * fx, bot = c + (d - c) * fx;
        return top + (bot - top) * fy;
    }

    private static float Ncc(GrayFrame cur, int x0, int y0, float[] tpl, float tmean, float tnorm, int P)
    {
        int n = cur.Width;
        if (x0 < 0 || y0 < 0 || x0 + P > n || y0 + P > cur.Height) return -2f;

        double sumI = 0, sumI2 = 0, dot = 0;
        for (int j = 0; j < P; j++)
        {
            int row = (y0 + j) * n + x0;
            int trow = j * P;
            for (int i = 0; i < P; i++)
            {
                int iv = cur.Pixels[row + i];
                sumI += iv; sumI2 += (double)iv * iv;
                dot += (double)iv * tpl[trow + i];
            }
        }
        int cnt = P * P;
        double meanI = sumI / cnt;
        double normI = Math.Sqrt(Math.Max(1e-6, sumI2 - sumI * sumI / cnt));
        double cov = dot - cnt * meanI * tmean;
        double denom = normI * tnorm;
        return denom > 1e-6 ? (float)(cov / denom) : -2f;
    }
}
