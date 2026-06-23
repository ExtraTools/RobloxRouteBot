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
}

/// <summary>
/// Главный цикл бота на отдельном потоке. На каждом тике:
///   позиция = provider.Update(прошлое направление) → контроллер даёт новое направление →
///   маппер → SendInput. Тикаем фиксированно ~60 Гц с высокоточным таймером.
/// Перед стартом — обратный отсчёт, чтобы успеть кликнуть в окно игры (ввод идёт в активное окно).
/// </summary>
public sealed class BotEngine
{
    private readonly InputSender _input;
    private readonly PurePursuitController _controller = new();

    private Thread? _thread;
    private volatile bool _running;

    public float Speed { get; set; } = 120f;     // мировых единиц/сек (для open-loop)
    public bool Loop { get; set; }
    public int StartDelayMs { get; set; } = 2000;
    public double TickHz { get; set; } = 60.0;

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
        _thread?.Join(1000);
        _thread = null;
        _input.ReleaseAll();
    }

    private void Run(IReadOnlyList<Vector2> route, IPositionProvider provider)
    {
        using var timer = new PreciseTimer();
        try
        {
            // Обратный отсчёт.
            for (int s = StartDelayMs / 1000; s > 0 && _running; s--)
            {
                Status?.Invoke($"Старт через {s}… кликни в окно игры!");
                Thread.Sleep(1000);
            }
            if (!_running) return;
            Status?.Invoke("Поехали.");

            double fixedDt = 1.0 / Math.Max(20.0, TickHz);
            double next = timer.ElapsedSeconds;
            double last = timer.ElapsedSeconds;
            Vector2 lastDir = Vector2.Zero;

            while (_running)
            {
                double now = timer.ElapsedSeconds;
                double dt = Math.Clamp(now - last, 0.0, 0.1);
                last = now;

                Vector2 pos = provider.Update(lastDir, Speed, dt);
                var (dir, done) = _controller.Compute(pos);

                if (done)
                {
                    if (Loop)
                    {
                        _controller.Reset();
                        provider.Start(route[0]);
                        Status?.Invoke("Круг пройден — повтор.");
                        lastDir = Vector2.Zero;
                        _input.SetHeld(MoveKey.None);
                    }
                    else
                    {
                        Status?.Invoke("Финиш.");
                        break;
                    }
                }
                else
                {
                    MoveKey keys = MovementMapper.Map(dir);
                    _input.SetHeld(keys);
                    lastDir = dir;
                    Tick?.Invoke(new TickInfo
                    {
                        Position = pos,
                        Direction = dir,
                        Keys = keys,
                        Confidence = provider.Confidence,
                    });
                }

                next += fixedDt;
                // если отстали (лаг) — не накапливаем долг
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
}
