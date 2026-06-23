using System.Numerics;
using RobloxRouteBot.Input;
using RobloxRouteBot.Vision;

namespace RobloxRouteBot.Core;

/// <summary>
/// Авто-калибровка базиса WASD→экран методом «нажми и измерь». Коротко жмём W, фазовой корреляцией
/// меряем, в какую сторону уехал мир на экране, → канвас-направление движения персонажа при W.
/// То же для D. Из двух направлений строим базис (FrameTransform), который заменяет любые ручные
/// инверсии/повороты и работает при ЛЮБОЙ ориентации камеры. Выполняется синхронно на потоке движка.
/// </summary>
public sealed class WasdCalibrator
{
    public int PressMs { get; set; } = 280;
    public int SampleStepMs { get; set; } = 25;
    public int SettleMs { get; set; } = 140;
    public float MinShiftPx { get; set; } = 1.2f;
    public float MinSampleConf { get; set; } = 0.05f;

    public bool Run(ICaptureSource cap, IPoseEstimator est, InputSender input, FrameTransform ft,
                    int n, Action<string>? status, Func<bool> alive)
    {
        ft.ResetHeading(); // базис меряем в неповёрнутом кадре (камера во время калибровки неподвижна)
        status?.Invoke("Калибровка: жму W…");
        if (!Measure(MoveKey.Forward, cap, est, input, ft, n, alive, out var fwd))
        {
            status?.Invoke("Калибровка W не удалась (упор в стену/нет сигнала). Базис по умолчанию.");
            input.SetHeld(MoveKey.None);
            return false;
        }

        status?.Invoke("Калибровка: жму D…");
        if (!Measure(MoveKey.Right, cap, est, input, ft, n, alive, out var str))
        {
            status?.Invoke("Калибровка D не удалась. Базис по умолчанию.");
            input.SetHeld(MoveKey.None);
            return false;
        }

        ft.SetBasis(fwd, str);
        input.SetHeld(MoveKey.None);
        if (ft.Calibrated)
        {
            status?.Invoke($"Базис откалиброван: W→({fwd.X:0.00},{fwd.Y:0.00}) D→({str.X:0.00},{str.Y:0.00}).");
            return true;
        }
        status?.Invoke("Базис вырожден (W и D почти совпали). По умолчанию.");
        return false;
    }

    private bool Measure(MoveKey key, ICaptureSource cap, IPoseEstimator est, InputSender input,
                         FrameTransform ft, int n, Func<bool> alive, out Vector2 dirCanvas)
    {
        dirCanvas = Vector2.Zero;
        est.Reset();

        // Прайм: один кадр до нажатия (станет «предыдущим»).
        var g0 = cap.GetGray(n, n, out _);
        if (g0 != null) est.Submit(g0);

        input.SetHeld(key);

        Vector2 net = Vector2.Zero;
        int samples = 0;
        int elapsed = 0;
        while (alive() && elapsed < PressMs)
        {
            Thread.Sleep(SampleStepMs);
            elapsed += SampleStepMs;
            var g = cap.GetGray(n, n, out bool dup);
            if (g == null || dup) continue;
            var pd = est.Submit(g);
            if (pd.Conf >= MinSampleConf) { net += pd.Shift; samples++; }
        }

        input.SetHeld(MoveKey.None);
        Thread.Sleep(SettleMs);

        if (!alive() || samples < 2 || net.Length() < MinShiftPx) return false;

        Vector2 world = ft.MapShiftToWorld(net); // канвас-смещение персонажа за нажатие
        if (world.LengthSquared() < 1e-6f) return false;
        dirCanvas = world;
        return true;
    }
}
