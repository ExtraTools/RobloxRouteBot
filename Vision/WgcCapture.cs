using System.Runtime.InteropServices;
using Microsoft.Graphics.Canvas;
using Windows.Graphics;
using Windows.Graphics.Capture;
using Windows.Graphics.DirectX;

namespace RobloxRouteBot.Vision;

/// <summary>
/// Захват конкретного окна через Windows Graphics Capture (GPU). В отличие от GDI/PrintWindow,
/// снимает реальный DirectX-кадр окна и работает, даже если окно ПЕРЕКРЫТО (наш бот сверху).
/// Кадры приходят на фоновый поток (CreateFreeThreaded); последний кадр кладём в буфер под локом,
/// UI/движок читают его через TryGetLatest. Чтение пикселей из GPU-поверхности — через Win2D.
/// Всё в try/catch: при любой неудаче возвращаем false, наверху есть GDI-фолбэк.
/// </summary>
public sealed class WgcCapture : IDisposable
{
    private static readonly Guid IGraphicsCaptureItemIid = new("79C3F95B-31F7-4EC2-A464-632EF5D30760");
    private const DirectXPixelFormat Fmt = DirectXPixelFormat.B8G8R8A8UIntNormalized;

    private CanvasDevice? _device;
    private GraphicsCaptureItem? _item;
    private Direct3D11CaptureFramePool? _pool;
    private GraphicsCaptureSession? _session;

    private readonly object _gate = new();
    private byte[]? _latest;
    private int _w, _h;
    private SizeInt32 _poolSize;
    private volatile bool _running;

    public bool IsRunning => _running;

    public static bool IsSupported
    {
        get { try { return GraphicsCaptureSession.IsSupported(); } catch { return false; } }
    }

    public bool TryStart(IntPtr hwnd)
    {
        Stop();
        try
        {
            if (hwnd == IntPtr.Zero || !GraphicsCaptureSession.IsSupported()) return false;

            _device = CanvasDevice.GetSharedDevice();
            _item = CreateItemForWindow(hwnd);
            if (_item == null) return false;

            _poolSize = _item.Size;
            if (_poolSize.Width <= 0 || _poolSize.Height <= 0) return false;

            _pool = Direct3D11CaptureFramePool.CreateFreeThreaded(_device, Fmt, 2, _poolSize);
            _pool.FrameArrived += OnFrameArrived;

            _session = _pool.CreateCaptureSession(_item);
            TrySet(() => _session.IsCursorCaptureEnabled = false);

            _item.Closed += (_, _) => Stop();
            _session.StartCapture();
            _running = true;
            return true;
        }
        catch
        {
            Stop();
            return false;
        }
    }

    private static void TrySet(Action a) { try { a(); } catch { } }

    private void OnFrameArrived(Direct3D11CaptureFramePool sender, object args)
    {
        try
        {
            using var frame = sender.TryGetNextFrame();
            if (frame == null || _device == null) return;

            var cs = frame.ContentSize;
            if (cs.Width > 0 && cs.Height > 0 && (cs.Width != _poolSize.Width || cs.Height != _poolSize.Height))
            {
                _poolSize = cs;
                try { _pool?.Recreate(_device, Fmt, 2, _poolSize); } catch { }
            }

            using var bmp = CanvasBitmap.CreateFromDirect3D11Surface(_device, frame.Surface);
            int w = (int)bmp.SizeInPixels.Width;
            int h = (int)bmp.SizeInPixels.Height;
            if (w <= 0 || h <= 0) return;

            byte[] bytes = bmp.GetPixelBytes(); // BGRA, stride = w*4, top-down
            lock (_gate) { _latest = bytes; _w = w; _h = h; }
        }
        catch { }
    }

    public bool TryGetLatest(out byte[] bgra, out int w, out int h)
    {
        lock (_gate)
        {
            if (_latest == null) { bgra = Array.Empty<byte>(); w = h = 0; return false; }
            bgra = _latest; w = _w; h = _h; return true;
        }
    }

    public void Stop()
    {
        _running = false;
        try { _session?.Dispose(); } catch { }
        try { if (_pool != null) _pool.FrameArrived -= OnFrameArrived; } catch { }
        try { _pool?.Dispose(); } catch { }
        _session = null;
        _pool = null;
        _item = null;
        lock (_gate) { _latest = null; _w = _h = 0; }
    }

    public void Dispose() => Stop();

    [ComImport, Guid("3628E81B-3CAC-4C60-B7F4-23CE0E0C3356"),
     InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
    private interface IGraphicsCaptureItemInterop
    {
        IntPtr CreateForWindow([In] IntPtr window, [In] ref Guid iid);
        IntPtr CreateForMonitor([In] IntPtr monitor, [In] ref Guid iid);
    }

    private static GraphicsCaptureItem? CreateItemForWindow(IntPtr hwnd)
    {
        var factory = WinRT.ActivationFactory.Get("Windows.Graphics.Capture.GraphicsCaptureItem");
        var interop = factory.AsInterface<IGraphicsCaptureItemInterop>();
        Guid iid = IGraphicsCaptureItemIid;
        IntPtr abi = interop.CreateForWindow(hwnd, ref iid);
        if (abi == IntPtr.Zero) return null;
        try { return GraphicsCaptureItem.FromAbi(abi); }
        finally { Marshal.Release(abi); }
    }
}
