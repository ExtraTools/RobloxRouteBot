using System.Numerics;

namespace RobloxRouteBot.Vision;

/// <summary>
/// Реализация ICaptureSource: сначала пытается отдать кадр из общего WgcCapture (GPU, ловит окно
/// даже под ботом), при неудаче — GDI-фолбэк (требует, чтобы окно было видимо). Делает downscale в
/// серый кадр анализа и детект дубликатов кадра по дешёвой выборке пикселей.
/// </summary>
public sealed class UnifiedCaptureSource : ICaptureSource
{
    private readonly WgcCapture _wgc;
    private readonly ScreenCapture _gdi;
    private readonly Func<IntPtr> _hwnd;

    private string _mode = "—";
    private long _lastHash = long.MinValue;

    public UnifiedCaptureSource(WgcCapture sharedWgc, ScreenCapture sharedGdi, Func<IntPtr> hwndSource)
    {
        _wgc = sharedWgc;
        _gdi = sharedGdi;
        _hwnd = hwndSource;
    }

    public string Mode => _mode;

    public GrayFrame? GetGray(int targetW, int targetH, out bool isDuplicate)
    {
        isDuplicate = false;
        if (targetW <= 0 || targetH <= 0) return null;

        GrayFrame? frame = null;

        if (_wgc.IsRunning && _wgc.TryGetLatest(out var full, out int fw, out int fh)
            && fw > 0 && fh > 0 && full.Length >= fw * fh * 4)
        {
            frame = DownscaleToGray(full, fw, fh, targetW, targetH);
            _mode = "WGC";
        }
        else
        {
            IntPtr h = _hwnd();
            frame = _gdi.Capture(h, targetW, targetH);
            _mode = frame != null ? "GDI" : "—";
        }

        if (frame == null) return null;

        long hash = SampleHash(frame.Pixels);
        isDuplicate = hash == _lastHash;
        _lastHash = hash;
        return frame;
    }

    public bool TryGetBgra(out byte[] bgra, out int width, out int height)
    {
        if (_wgc.IsRunning && _wgc.TryGetLatest(out bgra, out width, out height)
            && width > 0 && height > 0 && bgra.Length >= width * height * 4)
        {
            _mode = "WGC";
            return true;
        }
        IntPtr h = _hwnd();
        if (Native.Win32.TryGetClientRectOnScreen(h, out var r) && r.Width > 4 && r.Height > 4)
        {
            int th = 480;
            int tw = Math.Clamp((int)Math.Round(th * (double)r.Width / r.Height), 160, 1280);
            var got = _gdi.CaptureBgra(h, tw, th);
            if (got != null) { bgra = got; width = tw; height = th; _mode = "GDI"; return true; }
        }
        bgra = Array.Empty<byte>(); width = height = 0; _mode = "—"; return false;
    }

    private static GrayFrame DownscaleToGray(byte[] src, int sw, int sh, int dw, int dh)
    {
        var f = new GrayFrame(dw, dh);
        for (int y = 0; y < dh; y++)
        {
            int sy = (int)((long)y * sh / dh);
            int srow = sy * sw * 4;
            int drow = y * dw;
            for (int x = 0; x < dw; x++)
            {
                int sx = (int)((long)x * sw / dw);
                int si = srow + sx * 4;
                int b = src[si], g = src[si + 1], r = src[si + 2];
                f.Pixels[drow + x] = (byte)((r * 299 + g * 587 + b * 114) / 1000);
            }
        }
        return f;
    }

    /// <summary>Дешёвый хэш по разреженной выборке — для детекта дубликатов кадра.</summary>
    private static long SampleHash(byte[] px)
    {
        long h = 1469598103934665603L; // FNV-ish
        int step = Math.Max(1, px.Length / 512);
        for (int i = 0; i < px.Length; i += step)
        {
            h ^= px[i];
            h *= 1099511628211L;
        }
        return h;
    }
}
