using System.Runtime.InteropServices;
using RobloxRouteBot.Native;

namespace RobloxRouteBot.Vision;

/// <summary>
/// Захват клиентской области окна игры в уменьшенный серый кадр.
/// Берём пиксели из desktop DC по экранным координатам окна (StretchBlt c HALFTONE) — так
/// захватывается реально отрисованное DirectX-содержимое, чего BitBlt из самого окна не даёт.
/// Это чтение картинки рабочего стола, не процесса игры → для анти-чита невидимо.
/// </summary>
public sealed class ScreenCapture
{
    private const int SRCCOPY = 0x00CC0020;
    private const int HALFTONE = 4;
    private const uint DIB_RGB_COLORS = 0;
    private const uint BI_RGB = 0;
    private const uint PW_RENDERFULLCONTENT = 0x00000002;

    /// <summary>Захватить окно hwnd в кадр targetW x targetH. Возвращает null, если окно не найдено/свернуто.</summary>
    /// <summary>Серый кадр для оптического потока.</summary>
    public GrayFrame? Capture(IntPtr hwnd, int targetW, int targetH)
    {
        var bgra = CaptureRaw(hwnd, targetW, targetH);
        if (bgra == null) return null;

        int count = targetW * targetH;
        var frame = new GrayFrame(targetW, targetH);
        for (int i = 0; i < count; i++)
        {
            int b = bgra[i * 4 + 0];
            int g = bgra[i * 4 + 1];
            int r = bgra[i * 4 + 2];
            frame.Pixels[i] = (byte)((r * 299 + g * 587 + b * 114) / 1000);
        }
        return frame;
    }

    /// <summary>Цветной кадр (BGRA, top-down, stride = w*4) для лайв-превью игры.</summary>
    public byte[]? CaptureBgra(IntPtr hwnd, int targetW, int targetH) => CaptureRaw(hwnd, targetW, targetH);

    private byte[]? CaptureRaw(IntPtr hwnd, int targetW, int targetH)
    {
        if (targetW <= 0 || targetH <= 0 || hwnd == IntPtr.Zero) return null;

        IntPtr hScreen = GetDC(IntPtr.Zero);
        if (hScreen == IntPtr.Zero) return null;

        IntPtr dstDC = IntPtr.Zero, dstBmp = IntPtr.Zero, dstOld = IntPtr.Zero;
        IntPtr srcDC = IntPtr.Zero, srcBmp = IntPtr.Zero, srcOld = IntPtr.Zero;
        try
        {
            dstDC = CreateCompatibleDC(hScreen);
            if (dstDC == IntPtr.Zero) return null;

            var bmi = new BITMAPINFO
            {
                biSize = 40u, // sizeof(BITMAPINFOHEADER)
                biWidth = targetW,
                biHeight = -targetH, // top-down
                biPlanes = 1,
                biBitCount = 32,
                biCompression = BI_RGB,
            };
            dstBmp = CreateDIBSection(dstDC, ref bmi, DIB_RGB_COLORS, out IntPtr pBits, IntPtr.Zero, 0);
            if (dstBmp == IntPtr.Zero || pBits == IntPtr.Zero) return null;
            dstOld = SelectObject(dstDC, dstBmp);
            SetStretchBltMode(dstDC, HALFTONE);
            SetBrushOrgEx(dstDC, 0, 0, IntPtr.Zero);

            var bgra = new byte[targetW * targetH * 4];
            bool got = false;

            // 1) PrintWindow: снимает САМО окно, даже если оно перекрыто (наш бот может быть сверху).
            if (Win32.TryGetWindowRect(hwnd, out var wr) && wr.Width > 0 && wr.Height > 0)
            {
                srcDC = CreateCompatibleDC(hScreen);
                srcBmp = CreateCompatibleBitmap(hScreen, wr.Width, wr.Height);
                if (srcDC != IntPtr.Zero && srcBmp != IntPtr.Zero)
                {
                    srcOld = SelectObject(srcDC, srcBmp);
                    if (PrintWindow(hwnd, srcDC, PW_RENDERFULLCONTENT))
                    {
                        StretchBlt(dstDC, 0, 0, targetW, targetH, srcDC, 0, 0, wr.Width, wr.Height, SRCCOPY);
                        Marshal.Copy(pBits, bgra, 0, bgra.Length);
                        got = !IsMostlyBlack(bgra); // некоторые окна PrintWindow рендерит чёрным
                    }
                }
            }

            // 2) Фолбэк: захват области экрана по позиции окна (требует, чтобы окно было видимо).
            if (!got && Win32.TryGetClientRectOnScreen(hwnd, out var cr) && cr.Width > 0 && cr.Height > 0)
            {
                StretchBlt(dstDC, 0, 0, targetW, targetH, hScreen, cr.Left, cr.Top, cr.Width, cr.Height, SRCCOPY);
                Marshal.Copy(pBits, bgra, 0, bgra.Length);
                got = true;
            }

            return got ? bgra : null;
        }
        finally
        {
            if (srcOld != IntPtr.Zero) SelectObject(srcDC, srcOld);
            if (srcBmp != IntPtr.Zero) DeleteObject(srcBmp);
            if (srcDC != IntPtr.Zero) DeleteDC(srcDC);
            if (dstOld != IntPtr.Zero) SelectObject(dstDC, dstOld);
            if (dstBmp != IntPtr.Zero) DeleteObject(dstBmp);
            if (dstDC != IntPtr.Zero) DeleteDC(dstDC);
            ReleaseDC(IntPtr.Zero, hScreen);
        }
    }

    private static bool IsMostlyBlack(byte[] bgra)
    {
        long sum = 0; int n = 0;
        for (int i = 0; i < bgra.Length; i += 401) { sum += bgra[i]; n++; }
        return n == 0 || sum / (double)n < 6.0;
    }

    [DllImport("user32.dll")] private static extern IntPtr GetDC(IntPtr hWnd);
    [DllImport("user32.dll")] private static extern int ReleaseDC(IntPtr hWnd, IntPtr hDC);
    [DllImport("user32.dll")] private static extern bool PrintWindow(IntPtr hWnd, IntPtr hdcBlt, uint nFlags);

    [DllImport("gdi32.dll")] private static extern IntPtr CreateCompatibleDC(IntPtr hdc);
    [DllImport("gdi32.dll")] private static extern IntPtr CreateCompatibleBitmap(IntPtr hdc, int w, int h);
    [DllImport("gdi32.dll")] private static extern bool DeleteDC(IntPtr hdc);
    [DllImport("gdi32.dll")] private static extern IntPtr SelectObject(IntPtr hdc, IntPtr h);
    [DllImport("gdi32.dll")] private static extern bool DeleteObject(IntPtr ho);
    [DllImport("gdi32.dll")] private static extern int SetStretchBltMode(IntPtr hdc, int mode);
    [DllImport("gdi32.dll")] private static extern bool SetBrushOrgEx(IntPtr hdc, int x, int y, IntPtr lppt);

    [DllImport("gdi32.dll")]
    private static extern bool StretchBlt(IntPtr hdcDest, int xDest, int yDest, int wDest, int hDest,
        IntPtr hdcSrc, int xSrc, int ySrc, int wSrc, int hSrc, int rop);

    [DllImport("gdi32.dll")]
    private static extern IntPtr CreateDIBSection(IntPtr hdc, ref BITMAPINFO pbmi, uint usage,
        out IntPtr ppvBits, IntPtr hSection, uint offset);

    [StructLayout(LayoutKind.Sequential)]
    private struct BITMAPINFO
    {
        public uint biSize;
        public int biWidth;
        public int biHeight;
        public ushort biPlanes;
        public ushort biBitCount;
        public uint biCompression;
        public uint biSizeImage;
        public int biXPelsPerMeter;
        public int biYPelsPerMeter;
        public uint biClrUsed;
        public uint biClrImportant;
        // Цветовая таблица не нужна для 32bpp BI_RGB, но оставим место под один RGBQUAD.
        public uint bmiColors0;
    }
}
