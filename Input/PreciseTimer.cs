using System.Diagnostics;
using System.Runtime.InteropServices;

namespace RobloxRouteBot.Input;

/// <summary>
/// Высокоточное ожидание. Главная причина дрейфа дешёвых рекордеров — Thread.Sleep врёт
/// на ~15 мс и плавает. Здесь: timeBeginPeriod(1) повышает разрешение системного таймера,
/// а у дедлайна добавлен busy-spin на Stopwatch (точность до десятков микросекунд).
/// </summary>
public sealed class PreciseTimer : IDisposable
{
    [DllImport("winmm.dll")] private static extern uint timeBeginPeriod(uint uMilliseconds);
    [DllImport("winmm.dll")] private static extern uint timeEndPeriod(uint uMilliseconds);

    private readonly Stopwatch _sw = Stopwatch.StartNew();
    private bool _periodSet;

    public PreciseTimer()
    {
        if (timeBeginPeriod(1) == 0) _periodSet = true;
    }

    public double ElapsedSeconds => _sw.Elapsed.TotalSeconds;

    /// <summary>Спать (грубо), потом крутиться (точно) до достижения absolute-времени targetSeconds.</summary>
    public void SleepUntil(double targetSeconds)
    {
        while (true)
        {
            double remaining = targetSeconds - _sw.Elapsed.TotalSeconds;
            if (remaining <= 0) return;
            if (remaining > 0.002)
            {
                // оставляем 1.5 мс на спин, остальное спим
                int ms = (int)((remaining - 0.0015) * 1000);
                if (ms > 0) Thread.Sleep(ms);
            }
            else
            {
                Thread.SpinWait(50);
            }
        }
    }

    public void Dispose()
    {
        if (_periodSet)
        {
            timeEndPeriod(1);
            _periodSet = false;
        }
    }
}
