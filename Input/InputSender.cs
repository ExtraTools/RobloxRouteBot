using System.Runtime.InteropServices;

namespace RobloxRouteBot.Input;

/// <summary>
/// Логические клавиши движения. Маппинг на скан-коды — ниже.
/// </summary>
[Flags]
public enum MoveKey
{
    None = 0,
    Forward = 1 << 0, // W
    Back = 1 << 1,    // S
    Left = 1 << 2,    // A
    Right = 1 << 3,   // D
    Jump = 1 << 4,    // Space
}

/// <summary>
/// Отправка ввода через SendInput скан-кодами (KEYEVENTF_SCANCODE).
/// Скан-коды важны: многие игры (вкл. часть Roblox-ввода через raw input/DirectInput)
/// читают именно hardware scancode, а не виртуальную клавишу.
/// Класс держит набор «зажатых» клавиш и шлёт только дельту (нажать новые / отпустить ушедшие).
/// </summary>
public sealed class InputSender
{
    private const uint INPUT_KEYBOARD = 1;
    private const uint KEYEVENTF_KEYUP = 0x0002;
    private const uint KEYEVENTF_SCANCODE = 0x0008;

    // Скан-коды (set 1)
    private const ushort SC_W = 0x11;
    private const ushort SC_A = 0x1E;
    private const ushort SC_S = 0x1F;
    private const ushort SC_D = 0x20;
    private const ushort SC_SPACE = 0x39;

    private MoveKey _held = MoveKey.None;
    private readonly object _gate = new();

    private static ushort ScanFor(MoveKey k) => k switch
    {
        MoveKey.Forward => SC_W,
        MoveKey.Back => SC_S,
        MoveKey.Left => SC_A,
        MoveKey.Right => SC_D,
        MoveKey.Jump => SC_SPACE,
        _ => 0,
    };

    private static readonly MoveKey[] AllKeys =
    {
        MoveKey.Forward, MoveKey.Back, MoveKey.Left, MoveKey.Right, MoveKey.Jump,
    };

    /// <summary>Привести физическое состояние клавиатуры к заданному набору зажатых клавиш.</summary>
    public void SetHeld(MoveKey desired)
    {
        lock (_gate)
        {
            var inputs = new List<INPUT>(8);
            foreach (var k in AllKeys)
            {
                bool wantDown = desired.HasFlag(k);
                bool isDown = _held.HasFlag(k);
                if (wantDown == isDown) continue;
                inputs.Add(MakeKey(ScanFor(k), down: wantDown));
            }
            if (inputs.Count > 0)
            {
                SendInput((uint)inputs.Count, inputs.ToArray(), Marshal.SizeOf<INPUT>());
                _held = desired;
            }
        }
    }

    /// <summary>Отпустить всё. Вызывать при стопе/выходе, иначе клавиша «залипнет».</summary>
    public void ReleaseAll() => SetHeld(MoveKey.None);

    /// <summary>Относительное движение мыши (для будущей модели с поворотом камеры). В v1 не используется.</summary>
    public void MoveMouseRelative(int dx, int dy)
    {
        var input = new INPUT
        {
            type = 0, // INPUT_MOUSE
            U = new InputUnion
            {
                mi = new MOUSEINPUT
                {
                    dx = dx,
                    dy = dy,
                    dwFlags = 0x0001, // MOUSEEVENTF_MOVE
                },
            },
        };
        SendInput(1, new[] { input }, Marshal.SizeOf<INPUT>());
    }

    private static INPUT MakeKey(ushort scan, bool down) => new()
    {
        type = INPUT_KEYBOARD,
        U = new InputUnion
        {
            ki = new KEYBDINPUT
            {
                wVk = 0,
                wScan = scan,
                dwFlags = KEYEVENTF_SCANCODE | (down ? 0 : KEYEVENTF_KEYUP),
                time = 0,
                dwExtraInfo = IntPtr.Zero,
            },
        },
    };

    [DllImport("user32.dll", SetLastError = true)]
    private static extern uint SendInput(uint nInputs, INPUT[] pInputs, int cbSize);

    [StructLayout(LayoutKind.Sequential)]
    private struct INPUT
    {
        public uint type;
        public InputUnion U;
    }

    [StructLayout(LayoutKind.Explicit)]
    private struct InputUnion
    {
        [FieldOffset(0)] public MOUSEINPUT mi;
        [FieldOffset(0)] public KEYBDINPUT ki;
        [FieldOffset(0)] public HARDWAREINPUT hi;
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct MOUSEINPUT
    {
        public int dx;
        public int dy;
        public uint mouseData;
        public uint dwFlags;
        public uint time;
        public IntPtr dwExtraInfo;
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct KEYBDINPUT
    {
        public ushort wVk;
        public ushort wScan;
        public uint dwFlags;
        public uint time;
        public IntPtr dwExtraInfo;
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct HARDWAREINPUT
    {
        public uint uMsg;
        public ushort wParamL;
        public ushort wParamH;
    }
}
