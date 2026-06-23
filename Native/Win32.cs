using System.Runtime.InteropServices;
using System.Text;

namespace RobloxRouteBot.Native;

/// <summary>
/// Тонкая обёртка над WinAPI: найти окно игры, узнать его прямоугольник на экране, дать фокус.
/// Захват экрана делаем именно из desktop DC по координатам окна — так берутся реальные
/// (скомпонованные) пиксели DirectX-окна Roblox, чего BitBlt из самого окна обычно не даёт.
/// </summary>
public static class Win32
{
    public readonly record struct Rect(int Left, int Top, int Right, int Bottom)
    {
        public int Width => Right - Left;
        public int Height => Bottom - Top;
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct RECT
    {
        public int Left, Top, Right, Bottom;
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct POINT
    {
        public int X, Y;
    }

    private delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);

    [DllImport("user32.dll")]
    private static extern bool EnumWindows(EnumWindowsProc lpEnumFunc, IntPtr lParam);

    [DllImport("user32.dll", CharSet = CharSet.Unicode)]
    private static extern int GetWindowTextW(IntPtr hWnd, StringBuilder lpString, int nMaxCount);

    [DllImport("user32.dll")]
    private static extern int GetWindowTextLength(IntPtr hWnd);

    [DllImport("user32.dll")]
    private static extern bool IsWindowVisible(IntPtr hWnd);

    [DllImport("user32.dll")]
    private static extern bool GetWindowRect(IntPtr hWnd, out RECT lpRect);

    [DllImport("user32.dll")]
    private static extern bool GetClientRect(IntPtr hWnd, out RECT lpRect);

    [DllImport("user32.dll")]
    private static extern bool ClientToScreen(IntPtr hWnd, ref POINT lpPoint);

    [DllImport("user32.dll")]
    private static extern bool SetForegroundWindow(IntPtr hWnd);

    [DllImport("user32.dll")]
    private static extern IntPtr GetForegroundWindow();

    /// <summary>Ищет первое видимое окно, заголовок которого содержит titleSubstring (без регистра).</summary>
    public static IntPtr FindWindowByTitle(string titleSubstring)
    {
        IntPtr found = IntPtr.Zero;
        EnumWindows((h, _) =>
        {
            if (!IsWindowVisible(h)) return true;
            int len = GetWindowTextLength(h);
            if (len <= 0) return true;
            var sb = new StringBuilder(len + 1);
            GetWindowTextW(h, sb, sb.Capacity);
            string title = sb.ToString();
            if (title.Contains(titleSubstring, StringComparison.OrdinalIgnoreCase))
            {
                found = h;
                return false; // стоп
            }
            return true;
        }, IntPtr.Zero);
        return found;
    }

    /// <summary>Клиентская область окна в экранных координатах (без рамки/заголовка).</summary>
    public static bool TryGetClientRectOnScreen(IntPtr hWnd, out Rect rect)
    {
        rect = default;
        if (hWnd == IntPtr.Zero) return false;
        if (!GetClientRect(hWnd, out RECT cr)) return false;
        var topLeft = new POINT { X = cr.Left, Y = cr.Top };
        if (!ClientToScreen(hWnd, ref topLeft)) return false;
        rect = new Rect(topLeft.X, topLeft.Y, topLeft.X + (cr.Right - cr.Left), topLeft.Y + (cr.Bottom - cr.Top));
        return rect.Width > 0 && rect.Height > 0;
    }

    public static bool TryGetWindowRect(IntPtr hWnd, out Rect rect)
    {
        rect = default;
        if (hWnd == IntPtr.Zero || !GetWindowRect(hWnd, out RECT r)) return false;
        rect = new Rect(r.Left, r.Top, r.Right, r.Bottom);
        return rect.Width > 0 && rect.Height > 0;
    }

    public static void BringToFront(IntPtr hWnd)
    {
        if (hWnd != IntPtr.Zero) SetForegroundWindow(hWnd);
    }

    public static bool IsForeground(IntPtr hWnd) => hWnd != IntPtr.Zero && GetForegroundWindow() == hWnd;

    public readonly record struct WindowInfo(IntPtr Hwnd, string Title)
    {
        public override string ToString() => Title;
    }

    private const int GWL_EXSTYLE = -20;
    private const long WS_EX_TOOLWINDOW = 0x00000080;
    private const uint GW_OWNER = 4;

    [DllImport("user32.dll", EntryPoint = "GetWindowLongPtrW")]
    private static extern IntPtr GetWindowLongPtr(IntPtr hWnd, int nIndex);

    [DllImport("user32.dll")]
    private static extern IntPtr GetWindow(IntPtr hWnd, uint uCmd);

    [DllImport("kernel32.dll")]
    private static extern uint GetCurrentProcessId();

    [DllImport("user32.dll")]
    private static extern uint GetWindowThreadProcessId(IntPtr hWnd, out uint lpdwProcessId);

    /// <summary>
    /// Перечислить «настоящие» верхнеуровневые окна (alt-tab-able): видимые, с заголовком,
    /// без владельца, не tool-window, не наши собственные. Для выбора окна игры из списка.
    /// </summary>
    public static List<WindowInfo> ListWindows()
    {
        var list = new List<WindowInfo>();
        uint self = GetCurrentProcessId();

        EnumWindows((h, _) =>
        {
            if (!IsWindowVisible(h)) return true;
            if (GetWindow(h, GW_OWNER) != IntPtr.Zero) return true; // окно с владельцем (диалог/попап)

            long ex = (long)GetWindowLongPtr(h, GWL_EXSTYLE);
            if ((ex & WS_EX_TOOLWINDOW) != 0) return true;

            GetWindowThreadProcessId(h, out uint pid);
            if (pid == self) return true; // не показываем самих себя

            int len = GetWindowTextLength(h);
            if (len <= 0) return true;
            var sb = new StringBuilder(len + 1);
            GetWindowTextW(h, sb, sb.Capacity);
            string title = sb.ToString().Trim();
            if (title.Length == 0) return true;

            list.Add(new WindowInfo(h, title));
            return true;
        }, IntPtr.Zero);

        return list;
    }
}
