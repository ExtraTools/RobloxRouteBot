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
    lm.Harvest(A, pos, ft);
    Check("ориентиры засеялись", lm.Count > 0, $"count {lm.Count}");

    int dx = 8, dy = 8;
    var B = ShiftToroidal(A, dx, dy);                 // картинка уехала на (dx,dy)
    var newPos = pos + ft.MapShiftToWorld(new Vector2(dx, dy)); // перс уехал на (-dx,-dy)
    bool ok = lm.TryRelocalize(B, newPos, ft, out var posAbs, out var conf);
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

Console.WriteLine();
Console.WriteLine($"ИТОГ: {passed} passed, {failed} failed");
return failed == 0 ? 0 : 1;
