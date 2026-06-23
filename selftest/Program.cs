using System.Numerics;
using RobloxRouteBot.Core;
using RobloxRouteBot.Vision;

// Тесты математического ядра без живой игры: синтезируем кадры с ИЗВЕСТНЫМ сдвигом и гоняем
// реальный код. Ловит ошибки знака/координат — главный риск, который иначе виден только в игре.

int passed = 0, failed = 0;

void Check(string name, bool ok, string detail = "")
{
    if (ok) { passed++; Console.WriteLine($"  PASS  {name}  {detail}"); }
    else { failed++; Console.WriteLine($"X FAIL  {name}  {detail}"); }
}

static GrayFrame MakeTexture(int n, int seed)
{
    var f = new GrayFrame(n, n);
    for (int y = 0; y < n; y++)
        for (int x = 0; x < n; x++)
        {
            double v = 128
                + 55 * Math.Sin(x * 0.083) * Math.Cos(y * 0.061)
                + 38 * Math.Sin((x + y) * 0.131)
                + 25 * Math.Cos((x - 2 * y) * 0.047);
            f.Pixels[y * n + x] = (byte)Math.Clamp(v, 0, 255);
        }
    var rnd = new Random(seed);
    for (int k = 0; k < 40; k++)
    {
        int bx = rnd.Next(n), by = rnd.Next(n), br = rnd.Next(3, 9), bv = rnd.Next(0, 256);
        for (int dy = -br; dy <= br; dy++)
            for (int dx = -br; dx <= br; dx++)
                if (dx * dx + dy * dy <= br * br)
                {
                    int xx = ((bx + dx) % n + n) % n, yy = ((by + dy) % n + n) % n;
                    f.Pixels[yy * n + xx] = (byte)bv;
                }
    }
    return f;
}

// cur[x,y] = src[x-dx, y-dy] (тороидально) => cur = src, сдвинутый на (dx,dy).
static GrayFrame ShiftToroidal(GrayFrame src, int dx, int dy)
{
    int n = src.Width;
    var f = new GrayFrame(n, n);
    for (int y = 0; y < n; y++)
        for (int x = 0; x < n; x++)
        {
            int sx = ((x - dx) % n + n) % n, sy = ((y - dy) % n + n) % n;
            f.Pixels[y * n + x] = src.Pixels[sy * n + sx];
        }
    return f;
}

Console.WriteLine("== Phase correlation: знак и величина сдвига ==");
{
    int N = 256;
    var prev = MakeTexture(N, 1);
    foreach (var (dx, dy) in new[] { (7, 4), (-6, 9), (11, -5) })
    {
        var est = new PhaseCorrelationEstimator();
        est.Submit(prev);
        var cur = ShiftToroidal(prev, dx, dy);
        var (shift, conf) = est.Submit(cur);
        bool ok = Math.Abs(shift.X - dx) < 1.5f && Math.Abs(shift.Y - dy) < 1.5f && conf > 0.2f;
        Check($"shift ({dx},{dy})", ok, $"=> ({shift.X:0.0},{shift.Y:0.0}) conf {conf:0.00}");
    }
}

Console.WriteLine("== FrameTransform: знак интеграции (перс едет ПРОТИВ картинки) ==");
{
    var ft = new FrameTransform { ScaleX = 2f, ScaleY = 2f, InvertMotion = false };
    var w = ft.MapShiftToWorld(new Vector2(0, 5));   // картинка вниз => перс вверх => (0,-10)
    var d = ft.MapShiftToWorld(new Vector2(-5, 0));  // картинка влево => перс вправо => (+10,0)
    Check("картинка вниз → перс вверх", Math.Abs(w.X) < 0.01 && Math.Abs(w.Y + 10) < 0.01, $"=> ({w.X:0.0},{w.Y:0.0})");
    Check("картинка влево → перс вправо", Math.Abs(d.X - 10) < 0.01 && Math.Abs(d.Y) < 0.01, $"=> ({d.X:0.0},{d.Y:0.0})");
}

Console.WriteLine("== FrameTransform: инверсия базиса (направление → клавиши) ==");
{
    var ft = new FrameTransform();
    ft.SetBasis(new Vector2(0, -1), new Vector2(1, 0)); // W=вверх, D=вправо (норм. камера)
    var (fU, sU) = ft.WorldDirToBodyCoeffs(new Vector2(0, -1)); // вверх → W
    var (fR, sR) = ft.WorldDirToBodyCoeffs(new Vector2(1, 0));  // вправо → D
    var (fD, sD) = ft.WorldDirToBodyCoeffs(new Vector2(0, 1));  // вниз → S
    Check("вверх → forward+", fU > 0.5 && Math.Abs(sU) < 0.1, $"fwd {fU:0.0} str {sU:0.0}");
    Check("вправо → strafe+", Math.Abs(fR) < 0.1 && sR > 0.5, $"fwd {fR:0.0} str {sR:0.0}");
    Check("вниз → forward-", fD < -0.5 && Math.Abs(sD) < 0.1, $"fwd {fD:0.0} str {sD:0.0}");
}

Console.WriteLine("== Сквозная согласованность: калибровка W/D ↔ живая интеграция ==");
{
    // Имитируем: при W картинка уехала вниз (0,+6); при D — влево (-6,0).
    var ft = new FrameTransform { ScaleX = 1f, ScaleY = 1f };
    var fwdDir = ft.MapShiftToWorld(new Vector2(0, 6));   // (0,-6) => вверх
    var strDir = ft.MapShiftToWorld(new Vector2(-6, 0));  // (+6,0) => вправо
    ft.SetBasis(fwdDir, strDir);
    // Чтобы идти вверх, должен жаться W (forward+):
    var (f1, s1) = ft.WorldDirToBodyCoeffs(Vector2.Normalize(new Vector2(0, -1)));
    Check("W двигает вверх ⇒ чтобы вверх жмём W", f1 > 0.5 && Math.Abs(s1) < 0.2, $"fwd {f1:0.0} str {s1:0.0}");
}

Console.WriteLine("== LandmarkMap: harvest + relocalize (внутренняя согласованность) ==");
{
    int N = 256;
    var ft = new FrameTransform { ScaleX = 1f, ScaleY = 1f };
    var A = MakeTexture(N, 2);
    var lm = new LandmarkMap();
    var pos = new Vector2(1000, 1000);
    lm.Harvest(A, pos, ft, 0f);
    Check("ориентиры засеялись", lm.Count > 0, $"count {lm.Count}");

    int dx = 8, dy = 8;
    var B = ShiftToroidal(A, dx, dy);                 // картинка уехала на (dx,dy)
    var newPos = pos + ft.MapShiftToWorld(new Vector2(dx, dy)); // перс уехал на (-dx,-dy)
    bool ok = lm.TryRelocalize(B, newPos, ft, 0f, out var posAbs, out var conf);
    Check("relocalize нашёл ориентир", ok, $"conf {conf:0.00}");
    if (ok)
        Check("posAbs ≈ истинная позиция", Vector2.Distance(posAbs, newPos) < 4f,
            $"|Δ| {Vector2.Distance(posAbs, newPos):0.0} px");
}

Console.WriteLine("== PurePursuit: финиш по радиусу и по проекции ==");
{
    var pp = new PurePursuitController();
    var route = new List<Vector2> { new(0, 0), new(50, 0), new(100, 0) };
    pp.SetRoute(route);

    var (_, doneMid) = pp.Compute(new Vector2(20, 2));     // в начале — не финиш
    Check("в пути ≠ финиш", !doneMid);

    pp.SetRoute(route); // сброс курсора
    var (_, doneRadius) = pp.Compute(new Vector2(105, 6));  // рядом с концом — финиш по радиусу
    Check("у конца → финиш (радиус)", doneRadius);

    pp.SetRoute(route);
    var (_, doneProj) = pp.Compute(new Vector2(130, 30));   // проскочил конец вбок — финиш по проекции
    Check("проскочил конец → финиш (проекция)", doneProj);
}

// Реалистичный поворот камеры: крутим БОЛЬШУЮ текстуру и берём центральный кроп N×N,
// чтобы не было чёрных углов (в живой игре кадр при повороте остаётся полным).
static GrayFrame RotatedCrop(int seed, float ang, int N)
{
    int M = N * 3 / 2;
    var big = MakeTexture(M, seed);
    var rot = ImageWarp.RotateAboutCenter(big, ang);
    var f = new GrayFrame(N, N);
    int off = (M - N) / 2;
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++)
            f.Pixels[y * N + x] = rot.At(off + x, off + y);
    return f;
}

Console.WriteLine("== Fourier-Mellin: восстановление ПОВОРОТА ==");
{
    int N = 256;
    foreach (int deg in new[] { 3, 7, 12, 20 })
    {
        float ang = deg * MathF.PI / 180f;
        var fm = new FourierMellinEstimator();
        fm.Submit(RotatedCrop(5, 0f, N));     // prev: неповёрнутый кроп
        var pd = fm.Submit(RotatedCrop(5, ang, N)); // cur: тот же мир, повёрнут на ang
        float gotDeg = pd.DTheta * 180f / MathF.PI;
        bool ok = Math.Abs(gotDeg - deg) < 3.5f && pd.Conf > 0.12f;
        Check($"поворот {deg}°", ok, $"=> {gotDeg:0.0}° conf {pd.Conf:0.00}");
    }
}

Console.WriteLine("== Fourier-Mellin: стабильность на неподвижном кадре ==");
{
    var prev = MakeTexture(256, 6);
    var fm = new FourierMellinEstimator();
    fm.Submit(prev);
    var pd = fm.Submit(prev); // тот же кадр
    Check("нет ложного поворота", Math.Abs(pd.DTheta) < 0.02f && pd.Shift.Length() < 1.0f,
        $"dθ {pd.DTheta * 180 / Math.PI:0.0}° shift {pd.Shift.Length():0.0}");
}

Console.WriteLine("== FrameTransform: знак интеграции с поворотом (heading=90°) ==");
{
    var ft = new FrameTransform { ScaleX = 2f, ScaleY = 2f };
    ft.SetHeading(MathF.PI / 2);
    var w = ft.RotateScreenShiftToWorld(new Vector2(0, 5)); // v=(0,10); R(-90)=( +y, -x )*... ждём (-10,0)
    Check("heading=90 → (-10,0)", Math.Abs(w.X + 10) < 0.01 && Math.Abs(w.Y) < 0.01, $"=> ({w.X:0.0},{w.Y:0.0})");
}

Console.WriteLine("== FrameTransform: round-trip world↔screen при повороте ==");
{
    var ft = new FrameTransform { ScaleX = 1.5f, ScaleY = 0.8f };
    float h = 0.4f;
    var wo = new Vector2(37, -19);
    var so = ft.WorldOffsetToScreen(wo, h);
    var back = ft.ScreenOffsetToWorld(so, h);
    Check("WorldOffset∘ScreenOffset = id", Vector2.Distance(back, wo) < 0.01f, $"|Δ| {Vector2.Distance(back, wo):0.000}");
}

Console.WriteLine("== LandmarkMap: ре-локализация под ПОВОРОТОМ камеры ==");
{
    int N = 256;
    var ft = new FrameTransform { ScaleX = 1f, ScaleY = 1f };
    var A = MakeTexture(N, 7);
    var lm = new LandmarkMap();
    var pos = new Vector2(1000, 1000);
    lm.Harvest(A, pos, ft, 0f);                  // запомнили при heading 0
    Check("ориентиры засеялись (rot)", lm.Count > 0, $"count {lm.Count}");

    float ang = 12f * MathF.PI / 180f;
    var B = ImageWarp.RotateAboutCenter(A, ang); // камеру повернули, перс на месте
    bool ok = lm.TryRelocalize(B, pos, ft, ang, out var posAbs, out var conf);
    Check("ориентир найден под поворотом", ok, $"conf {conf:0.00}");
    if (ok)
        Check("posAbs ≈ pos (перс не двигался)", Vector2.Distance(posAbs, pos) < 8f,
            $"|Δ| {Vector2.Distance(posAbs, pos):0.0} px");
}

Console.WriteLine("== IPM: ректификация наклона камеры ==");
{
    int N = 256;
    float fov = 70f * MathF.PI / 180f;

    var id = new InversePerspectiveMap();
    id.Build(N, fov, 0f);
    var A = MakeTexture(N, 11);
    var dst = new GrayFrame(N, N);
    id.Rectify(A, dst);
    bool same = true;
    for (int i = 0; i < N * N; i++) if (dst.Pixels[i] != A.Pixels[i]) { same = false; break; }
    Check("tilt=0 → identity (pass-through)", id.Identity && same);

    float th = 18f * MathF.PI / 180f;
    var ipmP = new InversePerspectiveMap(); ipmP.Build(N, fov, th);
    var ipmM = new InversePerspectiveMap(); ipmM.Build(N, fov, -th);
    var Hp = ipmP.Homography; var Hm = ipmM.Homography;
    var prod = new double[9];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            prod[r * 3 + c] = Hp[r * 3] * Hm[c] + Hp[r * 3 + 1] * Hm[3 + c] + Hp[r * 3 + 2] * Hm[6 + c];
    double[] I = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    double herr = 0;
    for (int i = 0; i < 9; i++) herr += Math.Abs(prod[i] / prod[8] - I[i]);
    Check("H(θ)·H(−θ) ≈ I", herr < 1e-3, $"err {herr:0.0000}");

    // Ректификация (как в бою — ОДИН ресэмпл) двух повёрнутых кадров → FM должен ловить поворот.
    var rec = new InversePerspectiveMap(); rec.Build(N, fov, 14f * MathF.PI / 180f);
    var bk1 = new GrayFrame(N, N); rec.Rectify(RotatedCrop(11, 0f, N), bk1);
    float aDeg = 12f;
    var bk2 = new GrayFrame(N, N); rec.Rectify(RotatedCrop(11, aDeg * MathF.PI / 180f, N), bk2);
    var fm = new FourierMellinEstimator(); fm.Submit(bk1);
    var pd = fm.Submit(bk2);
    float gotDeg = pd.DTheta * 180f / MathF.PI;
    Check("на ректифицированных кадрах FM ловит поворот", Math.Abs(gotDeg - aDeg) < 5f && pd.Conf > 0.1f,
        $"=> {gotDeg:0.0}° conf {pd.Conf:0.00}");
}

Console.WriteLine("== Fourier-Mellin: перформанс (256×256, путь с поворотом) ==");
{
    int N = 256;
    var fm = new FourierMellinEstimator();
    var frames = new GrayFrame[16];
    for (int i = 0; i < frames.Length; i++) frames[i] = RotatedCrop(9, i * 0.4f * MathF.PI / 180f, N);
    fm.Submit(frames[0]); // прогрев
    var sw = System.Diagnostics.Stopwatch.StartNew();
    int iters = 60;
    for (int i = 0; i < iters; i++) fm.Submit(frames[i % frames.Length]);
    sw.Stop();
    double ms = sw.Elapsed.TotalMilliseconds / iters;
    Check("FM Submit укладывается в реальное время", ms < 35.0, $"avg {ms:0.0} ms (~{1000.0 / ms:0} fps)");
}

Console.WriteLine();
Console.WriteLine($"ИТОГ: {passed} passed, {failed} failed");
return failed == 0 ? 0 : 1;
