using System.Numerics;
using RobloxRouteBot.Input;
using RobloxRouteBot.Vision;

namespace RobloxRouteBot.Core;

public sealed class TickInfo
{
    public Vector2 Position { get; init; }
    public Vector2 Direction { get; init; }
    public MoveKey Keys { get; init; }
    public float Confidence { get; init; }

    // Расширенная телеметрия (для лайв-оверлея). Заполняется для визуальной одометрии.
    public string Mode { get; init; } = "";
    public float Response { get; init; }
    public float DriftSinceSnap { get; init; }
    public bool Snapped { get; init; }
    public float Fps { get; init; }
    public int Landmarks { get; init; }
    public float HeadingDeg { get; init; }
    public string Phase { get; init; } = "RUN";
}

/// <summary>
/// Главный цикл бота на отдельном потоке. На каждом тике:
///   позиция = provider.Update(прошлое направление) → контроллер даёт новое направление →
///   маппер (через откалиброванный базис) → SendInput. Тикаем фиксированно ~60 Гц с высокоточным таймером.
/// Перед стартом — обратный отсчёт + (для зрения) авто-калибровка базиса WASD. Есть защита от
/// застревания/потери картинки: «финиш» не объявляется на мёртвом зрении.
/// </summary>
public sealed class BotEngine
{
    private readonly InputSender _input;
    private readonly PurePursuitController _controller = new();
    private readonly WasdCalibrator _calibrator = new();
    private readonly FrameTransform _defaultFrame = new();

    private Thread? _thread;
    private volatile bool _running;
    private IPositionProvider? _provider;

    public float Speed { get; set; } = 120f;
    public bool Loop { get; set; }
    public int StartDelayMs { get; set; } = 2000;
    public double TickHz { get; set; } = 60.0;
    public bool AutoCalibrate { get; set; } = true;

    /// <summary>Сколько секунд без прогресса при зажатых клавишах = «застрял».</summary>
    public double StuckSeconds { get; set; } = 5.0;
    /// <summary>Сколько секунд при нулевой уверенности = «потерял картинку».</summary>
    public double LostSeconds { get; set; } = 4.0;

    public PurePursuitController Controller => _controller;

    public event Action<TickInfo>? Tick;
    public event Action<string>? Status;
    public event Action? Stopped;

    public bool IsRunning => _running;

    public BotEngine(InputSender input) => _input = input;

    public void Start(IReadOnlyList<Vector2> route, IPositionProvider provider)
    {
        if (_running) return;
        if (route.Count < 2)
        {
            Status?.Invoke("Маршрут пустой — нарисуй хотя бы две точки.");
            return;
        }

        _provider = provider;
        _controller.SetRoute(route);
        provider.Start(route[0]);
        _running = true;

        _thread = new Thread(() => Run(route, provider))
        {
            IsBackground = true,
            Name = "BotEngine",
        };
        _thread.Start();
    }

    public void Stop()
    {
        _running = false;
        _thread?.Join(1500);
        _thread = null;
        _input.ReleaseAll();
    }

    /// <summary>Ручной ре-якорь позиции (перетаскивание маркера во время прогона).</summary>
    public void SetMeasuredPosition(Vector2 worldPos) => _provider?.SetPosition(worldPos);

    private FrameTransform FrameFor(IPositionProvider provider)
        => provider is VisualOdometryProvider vo ? vo.Frame : _defaultFrame;

    private void Run(IReadOnlyList<Vector2> route, IPositionProvider provider)
    {
        using var timer = new PreciseTimer();
        var rng = Random.Shared;
        var frame = FrameFor(provider);
        try
        {
            // Обратный отсчёт — успеть кликнуть в окно игры (ввод идёт в активное окно).
            for (int s = StartDelayMs / 1000; s > 0 && _running; s--)
            {
                Status?.Invoke($"Старт через {s}… кликни в окно игры!");
                Thread.Sleep(1000);
            }
            if (!_running) return;

            // Авто-калибровка базиса WASD по зрению.
            if (AutoCalibrate && provider is VisualOdometryProvider vo)
            {
                _calibrator.Run(vo.Capture, vo.PoseEstimator, _input, vo.Frame, vo.AnalysisSize,
                                Status, () => _running);
                if (!_running) return;
                provider.SetPosition(route[0]); // сброс после калибровочных движений
            }

            Status?.Invoke("Поехали.");

            double fixedDt = 1.0 / Math.Max(20.0, TickHz);
            double next = timer.ElapsedSeconds;
            double last = timer.ElapsedSeconds;
            Vector2 lastDir = Vector2.Zero;

            // Защита от застревания/потери.
            Vector2 progressAnchor = route[0];
            double progressTimer = 0;
            double lostTimer = 0;

            while (_running)
            {
                double now = timer.ElapsedSeconds;
                double dt = Math.Clamp(now - last, 0.0, 0.1);
                last = now;

                Vector2 pos = provider.Update(lastDir, Speed, dt);
                var (dir, done) = _controller.Compute(pos);

                // --- защита: потеря картинки ---
                if (provider.Confidence <= 0.02f) lostTimer += dt; else lostTimer = 0;
                if (lostTimer >= LostSeconds)
                {
                    Status?.Invoke("Потеряна картинка (нет сигнала зрения) — стоп. Проверь захват/камеру.");
                    break;
                }

                // --- защита: застревание (клавиши жмём, а позиция не растёт) ---
                if (lastDir != Vector2.Zero)
                {
                    if (Vector2.Distance(pos, progressAnchor) > 6f) { progressAnchor = pos; progressTimer = 0; }
                    else progressTimer += dt;
                    if (progressTimer >= StuckSeconds)
                    {
                        Status?.Invoke("Застрял (упор/нет продвижения) — стоп. Подвинь маркер или перезапусти.");
                        break;
                    }
                }
                else { progressAnchor = pos; progressTimer = 0; }

                if (done)
                {
                    if (Loop)
                    {
                        _controller.Reset();
                        provider.SetPosition(route[0]);
                        Status?.Invoke("Круг пройден — повтор.");
                        lastDir = Vector2.Zero;
                        progressAnchor = route[0]; progressTimer = 0;
                        _input.SetHeld(MoveKey.None);
                    }
                    else
                    {
                        Status?.Invoke("Финиш — персонаж дошёл.");
                        break;
                    }
                }
                else
                {
                    MoveKey keys = MovementMapper.Map(dir, frame);
                    _input.SetHeld(keys);
                    lastDir = dir;
                    Tick?.Invoke(BuildTick(provider, pos, dir, keys));
                }

                // Джиттер периода — против идеально периодического паттерна нажатий.
                next += fixedDt + (rng.NextDouble() - 0.5) * 0.004;
                if (next < timer.ElapsedSeconds) next = timer.ElapsedSeconds;
                timer.SleepUntil(next);
            }
        }
        finally
        {
            _input.ReleaseAll();
            _running = false;
            Status?.Invoke("Остановлен.");
            Stopped?.Invoke();
        }
    }

    private TickInfo BuildTick(IPositionProvider provider, Vector2 pos, Vector2 dir, MoveKey keys)
    {
        if (provider is VisualOdometryProvider vo)
        {
            return new TickInfo
            {
                Position = pos, Direction = dir, Keys = keys, Confidence = vo.Confidence,
                Mode = vo.CaptureMode, Response = vo.Response, DriftSinceSnap = vo.DriftSinceSnap,
                Snapped = vo.SnappedThisTick, Fps = vo.Fps, Landmarks = vo.LandmarkCount,
                HeadingDeg = vo.HeadingDeg, Phase = "RUN",
            };
        }
        return new TickInfo
        {
            Position = pos, Direction = dir, Keys = keys, Confidence = provider.Confidence,
            Mode = "", Phase = "RUN",
        };
    }
}
