using System.Collections.ObjectModel;
using System.Globalization;
using System.Numerics;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.Windows.Threading;
using RobloxRouteBot.Core;
using RobloxRouteBot.Input;
using RobloxRouteBot.Native;
using RobloxRouteBot.Vision;

namespace RobloxRouteBot;

public partial class MainWindow : Window
{
    private readonly InputSender _input = new();
    private readonly BotEngine _engine;
    private readonly OpticalFlowSettings _flowSettings = new();
    private readonly ObservableCollection<Win32.WindowInfo> _windows = new();

    private readonly List<Vector2> _route = new();
    private readonly Polyline _routeUnderlay;
    private readonly Polyline _routeLine;
    private readonly Ellipse _startRing, _startCore, _dot;
    private readonly Rectangle _endMarker;
    private readonly Line _dotHeading;

    private bool _drawing;
    private CalibrationWindow? _calibWin;
    private Storyboard? _pulse;

    private readonly ScreenCapture _liveCapture = new();
    private readonly WgcCapture _wgc = new();
    private DispatcherTimer? _liveTimer;
    private WriteableBitmap? _liveWb;
    private bool? _liveWasDark;

    private readonly FrameTransform _frame = new();
    private readonly UnifiedCaptureSource _captureSource;
    private bool _draggingMarker;

    public MainWindow()
    {
        InitializeComponent();

        _engine = new BotEngine(_input);
        _engine.Tick += OnTick;
        _engine.Status += SetStatus;
        _engine.Stopped += OnStopped;

        _captureSource = new UnifiedCaptureSource(_wgc, _liveCapture, SelectedHwnd);

        CmbWindow.ItemsSource = _windows;

        // Слои маршрута (порядок = z-order снизу вверх)
        _routeUnderlay = MakeLine(Brush("Accent"), 7) ; _routeUnderlay.Opacity = 0.16;
        _routeLine = MakeLine(Brush("Accent"), 2.5);
        _startRing = new Ellipse { Width = 14, Height = 14, Stroke = Brush("Accent"), StrokeThickness = 2.5, Fill = Brush("CanvasBg"), Visibility = Visibility.Collapsed };
        _startCore = new Ellipse { Width = 5, Height = 5, Fill = Brush("Accent"), Visibility = Visibility.Collapsed };
        _endMarker = new Rectangle { Width = 13, Height = 13, RadiusX = 3, RadiusY = 3, Stroke = Brush("Accent"), StrokeThickness = 2.5, Fill = Brush("CanvasBg"), Visibility = Visibility.Collapsed };
        _dotHeading = new Line { Stroke = Brush("Accent"), StrokeThickness = 2, Visibility = Visibility.Collapsed };
        _dot = new Ellipse { Width = 9, Height = 9, Fill = Brush("Dot"), Stroke = Brush("Accent"), StrokeThickness = 1, Visibility = Visibility.Collapsed };

        foreach (UIElement el in new UIElement[] { _routeUnderlay, _routeLine, _startRing, _startCore, _endMarker, _dotHeading, _dot })
            RouteCanvas.Children.Add(el);

        BuildPulse();
        RefreshWindows();

        StateChanged += (_, _) => RootBorder.Margin = WindowState == WindowState.Maximized ? new Thickness(7) : new Thickness(0);
        Closed += (_, _) => { _engine.Stop(); _liveTimer?.Stop(); _wgc.Dispose(); _calibWin?.Close(); };
    }

    private static Polyline MakeLine(Brush stroke, double thickness) => new()
    {
        Stroke = stroke,
        StrokeThickness = thickness,
        StrokeLineJoin = PenLineJoin.Round,
        StrokeStartLineCap = PenLineCap.Round,
        StrokeEndLineCap = PenLineCap.Round,
    };

    private SolidColorBrush Brush(string key) => (SolidColorBrush)FindResource(key);

    // ===== Скан окон =====

    private void RefreshWindows()
    {
        var sel = (CmbWindow.SelectedItem as Win32.WindowInfo?)?.Hwnd ?? IntPtr.Zero;
        _windows.Clear();
        foreach (var w in Win32.ListWindows().OrderBy(w => w.Title, StringComparer.OrdinalIgnoreCase))
            _windows.Add(w);

        // вернуть прошлый выбор, иначе автоселект Roblox, иначе первый
        int idx = -1;
        for (int i = 0; i < _windows.Count; i++)
            if (_windows[i].Hwnd == sel) { idx = i; break; }
        if (idx < 0)
            for (int i = 0; i < _windows.Count; i++)
                if (_windows[i].Title.Contains("Roblox", StringComparison.OrdinalIgnoreCase)) { idx = i; break; }
        if (idx < 0 && _windows.Count > 0) idx = 0;
        CmbWindow.SelectedIndex = idx;
        SetStatus($"Найдено окон: {_windows.Count}.");
    }

    private void BtnRefresh_Click(object sender, RoutedEventArgs e) => RefreshWindows();

    private IntPtr SelectedHwnd() => (CmbWindow.SelectedItem as Win32.WindowInfo?)?.Hwnd ?? IntPtr.Zero;

    // ===== Рисование =====

    private void Canvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
    {
        Point p = e.GetPosition(RouteCanvas);

        // Во время прогона ЛКМ по маркеру = ручной ре-якорь («вот где перс на самом деле»).
        if (_engine.IsRunning)
        {
            if (_dot.Visibility == Visibility.Visible && DistanceToDot(p) <= 16)
            {
                _draggingMarker = true;
                _dot.Width = _dot.Height = 14;
                RouteCanvas.CaptureMouse();
                SetStatus("Ре-якорь: тащи маркер на реальную позицию персонажа.");
            }
            return;
        }

        _drawing = true;
        ClearRoute();
        AddPoint(p);
        RouteCanvas.CaptureMouse();
    }

    private void Canvas_MouseMove(object sender, MouseEventArgs e)
    {
        if (_draggingMarker)
        {
            Place(_dot, ToVec(e.GetPosition(RouteCanvas)));
            return;
        }
        if (!_drawing) return;
        AddPoint(e.GetPosition(RouteCanvas));
    }

    private void Canvas_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
    {
        if (_draggingMarker)
        {
            _draggingMarker = false;
            _dot.Width = _dot.Height = 9;
            RouteCanvas.ReleaseMouseCapture();
            var v = ToVec(e.GetPosition(RouteCanvas));
            _engine.SetMeasuredPosition(v);
            SetStatus("Позиция переустановлена вручную.");
            return;
        }
        if (!_drawing) return;
        _drawing = false;
        RouteCanvas.ReleaseMouseCapture();
        RenderDecorations();
        SetStatus($"Маршрут готов: {_route.Count} точек. Жми «Запустить».");
    }

    private static Vector2 ToVec(Point p) => new((float)p.X, (float)p.Y);

    private double DistanceToDot(Point p)
    {
        double cx = Canvas.GetLeft(_dot) + _dot.Width / 2;
        double cy = Canvas.GetTop(_dot) + _dot.Height / 2;
        double dx = p.X - cx, dy = p.Y - cy;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    private void AddPoint(Point p)
    {
        var v = new Vector2((float)p.X, (float)p.Y);
        if (_route.Count > 0 && Vector2.Distance(_route[^1], v) < 8f) return;
        _route.Add(v);
        _routeLine.Points.Add(p);
        _routeUnderlay.Points.Add(p);
        UpdateRouteStat();
    }

    private void ClearRoute()
    {
        _route.Clear();
        _routeLine.Points.Clear();
        _routeUnderlay.Points.Clear();
        _startRing.Visibility = _startCore.Visibility = _endMarker.Visibility =
            _dot.Visibility = _dotHeading.Visibility = Visibility.Collapsed;
        UpdateRouteStat();
    }

    private void UpdateRouteStat()
    {
        double len = 0;
        for (int i = 1; i < _route.Count; i++) len += Vector2.Distance(_route[i - 1], _route[i]);
        TxtRouteStat.Text = $"{_route.Count} pts // {len:0} px";
    }

    private void RenderDecorations()
    {
        if (_route.Count >= 1)
        {
            Place(_startRing, _route[0]);
            Place(_startCore, _route[0]);
            _startRing.Visibility = _startCore.Visibility = Visibility.Visible;
        }
        if (_route.Count >= 2)
        {
            Place(_endMarker, _route[^1]);
            _endMarker.Visibility = Visibility.Visible;
        }
    }

    private static void Place(FrameworkElement el, Vector2 p)
    {
        Canvas.SetLeft(el, p.X - el.Width / 2);
        Canvas.SetTop(el, p.Y - el.Height / 2);
    }

    private void BtnClear_Click(object sender, RoutedEventArgs e)
    {
        if (_engine.IsRunning) return;
        ClearRoute();
        SetStatus("Очищено.");
    }

    // ===== Запуск =====

    private void BtnPlay_Click(object sender, RoutedEventArgs e)
    {
        if (_engine.IsRunning) return;
        if (_route.Count < 2) { SetStatus("Сначала нарисуй маршрут."); return; }

        IntPtr hwnd = SelectedHwnd();
        _engine.Speed = ParseFloat(TxtSpeed.Text, 120f);
        _engine.Loop = ChkLoop.IsChecked == true;

        IPositionProvider provider;
        if (RbVo.IsChecked == true)
        {
            if (hwnd == IntPtr.Zero) { SetStatus("Выбери окно игры («Скан») — зрению нужен источник кадров."); return; }
            if (!_wgc.IsRunning) _wgc.TryStart(hwnd); // даже без лайв-превью движку нужен захват

            float mult = ParseFloat(TxtPxScale.Text, 1f);
            double cw = RouteCanvas.ActualWidth > 16 ? RouteCanvas.ActualWidth : 512;
            double ch = RouteCanvas.ActualHeight > 16 ? RouteCanvas.ActualHeight : 512;
            _frame.ResetBasis();
            _frame.InvertMotion = false;
            _frame.ScaleX = (float)(cw / 256.0) * mult;
            _frame.ScaleY = (float)(ch / 256.0) * mult;

            provider = new VisualOdometryProvider(_captureSource, new PhaseCorrelationEstimator(), _frame, new LandmarkMap());
            SetStatus("Зрение: камеру держи строго сверху. После отсчёта бот сам калибрует W/D.");
        }
        else if (RbFlow.IsChecked == true)
        {
            if (hwnd == IntPtr.Zero) { SetStatus("Выбери окно игры (кнопка «Скан») — для оптического потока оно нужно."); return; }
            _flowSettings.WorldPerPixel = ParseFloat(TxtPxScale.Text, _flowSettings.WorldPerPixel);
            provider = new OpticalFlowProvider(SelectedHwnd, _flowSettings);
        }
        else
        {
            provider = new DeadReckoningProvider();
        }

        _dot.Visibility = Visibility.Visible;
        Place(_dot, _route[0]);
        SetUiRunning(true);
        _engine.Start(_route.ToArray(), provider);
    }

    private void BtnStop_Click(object sender, RoutedEventArgs e) => _engine.Stop();

    private void BtnCalib_Click(object sender, RoutedEventArgs e)
    {
        _flowSettings.WorldPerPixel = ParseFloat(TxtPxScale.Text, _flowSettings.WorldPerPixel);
        if (_calibWin != null) { _calibWin.Activate(); return; }
        _calibWin = new CalibrationWindow(_flowSettings, SelectedHwnd,
            scale => TxtPxScale.Text = scale.ToString("0.###", CultureInfo.InvariantCulture)) { Owner = this };
        _calibWin.Closed += (_, _) => _calibWin = null;
        _calibWin.Show();
    }

    // ===== События движка =====

    private void OnTick(TickInfo t)
    {
        Dispatcher.InvokeAsync(() =>
        {
            if (!_draggingMarker)
            {
                Place(_dot, t.Position);
                if (t.Direction.LengthSquared() > 1e-4f)
                {
                    var d = Vector2.Normalize(t.Direction);
                    _dotHeading.X1 = t.Position.X; _dotHeading.Y1 = t.Position.Y;
                    _dotHeading.X2 = t.Position.X + d.X * 13; _dotHeading.Y2 = t.Position.Y + d.Y * 13;
                    _dotHeading.Visibility = Visibility.Visible;
                }
            }
            TxtPos.Text = $"X:{t.Position.X,4:0} Y:{t.Position.Y,4:0} | {t.Keys} | conf {t.Confidence:0.00}";
            UpdateConfidence(t.Confidence);

            // Лайв-телеметрия.
            TxtTmMode.Text = string.IsNullOrEmpty(t.Mode) ? "—" : t.Mode;
            TxtTmFps.Text = t.Fps > 0 ? $"{t.Fps,4:0.0}" : "—";
            TxtTmResp.Text = $"{t.Response:0.00}";
            TxtTmDrift.Text = $"{t.DriftSinceSnap,4:0.0} px";
            TxtTmLm.Text = t.Landmarks.ToString();
            TxtTmKeys.Text = KeysGlyph(t.Keys);
            TxtTmPos.Text = $"{t.Position.X,4:0},{t.Position.Y,4:0}";
            TxtTmSnap.Opacity = t.Snapped ? 1.0 : Math.Max(0, TxtTmSnap.Opacity * 0.82);
        });
    }

    private static string KeysGlyph(MoveKey k) =>
        $"{(k.HasFlag(MoveKey.Forward) ? "W" : "·")}{(k.HasFlag(MoveKey.Left) ? "A" : "·")}" +
        $"{(k.HasFlag(MoveKey.Back) ? "S" : "·")}{(k.HasFlag(MoveKey.Right) ? "D" : "·")}" +
        $"{(k.HasFlag(MoveKey.Jump) ? " ⎵" : "")}";

    private void UpdateConfidence(float conf)
    {
        conf = Math.Clamp(conf, 0f, 1f);
        double inner = Math.Max(0, ConfTrack.ActualWidth - 2);
        ConfFill.Width = inner * conf;
        string key = conf < 0.35f ? "Danger" : conf < 0.70f ? "Amber" : "Success";
        ConfFill.Fill = Brush(key);
        TxtConfVal.Text = conf.ToString("0.00");
        TxtConfVal.Foreground = conf <= 0f ? Brush("TextMuted") : Brush(key);
    }

    private void SetStatus(string s) => Dispatcher.InvokeAsync(() => TxtStatus.Text = s);

    private void OnStopped() => Dispatcher.InvokeAsync(() => SetUiRunning(false));

    private void SetUiRunning(bool running)
    {
        BtnPlay.IsEnabled = !running;
        BtnStop.IsEnabled = running;
        BtnClear.IsEnabled = !running;
        BtnRefresh.IsEnabled = !running;
        // Hit-test НЕ глушим во время прогона — нужен для перетаскивания маркера (ре-якорь).
        // Рисование маршрута блокируется проверкой _engine.IsRunning в обработчиках.

        TxtRunState.Text = running ? "RUNNING" : "IDLE";
        TxtStateTag.Text = running ? "RUN" : "IDLE";
        RunDot.Fill = running ? Brush("Accent") : Brush("TextMuted");
        StatusDot.Background = running ? Brush("Accent") : Brush("TextMuted");

        if (running) _pulse?.Begin(this, true);
        else
        {
            _pulse?.Stop(this);
            StatusDot.Opacity = 1; RunDot.Opacity = 1;
        }
    }

    private void BuildPulse()
    {
        _pulse = new Storyboard { RepeatBehavior = RepeatBehavior.Forever, AutoReverse = true };
        foreach (var (target, prop) in new (DependencyObject, string)[] { (StatusDot, "Opacity"), (RunDot, "Opacity") })
        {
            var anim = new DoubleAnimation(1.0, 0.45, new Duration(TimeSpan.FromMilliseconds(1400)))
            {
                EasingFunction = new SineEase { EasingMode = EasingMode.EaseInOut },
            };
            Storyboard.SetTarget(anim, (DependencyObject)target);
            Storyboard.SetTargetProperty(anim, new PropertyPath(prop));
            _pulse.Children.Add(anim);
        }
    }

    // ===== Титлбар =====

    private void BtnMin_Click(object sender, RoutedEventArgs e) => WindowState = WindowState.Minimized;
    private void BtnClose_Click(object sender, RoutedEventArgs e) => Close();

    // ===== Лайв-превью игры =====

    private void ChkLive_Toggle(object sender, RoutedEventArgs e)
    {
        if (ChkLive.IsChecked == true) StartLive();
        else StopLive();
    }

    private void StartLive()
    {
        if (SelectedHwnd() == IntPtr.Zero)
        {
            SetStatus("Выбери окно игры («Скан») — нечего показывать.");
            ChkLive.IsChecked = false;
            return;
        }
        RouteCanvas.Background = System.Windows.Media.Brushes.Transparent;
        LiveImage.Visibility = Visibility.Visible;
        _liveWasDark = null;

        bool wgc = _wgc.TryStart(SelectedHwnd());
        SetStatus(wgc
            ? "Лайв через Windows Graphics Capture — бот можно держать поверх игры."
            : "Лайв через экранный захват — игра должна быть видима (не за этим окном).");

        if (_liveTimer == null)
        {
            _liveTimer = new DispatcherTimer(DispatcherPriority.Background) { Interval = TimeSpan.FromMilliseconds(33) };
            _liveTimer.Tick += LiveTick;
        }
        _liveTimer.Start();
    }

    private void StopLive()
    {
        _liveTimer?.Stop();
        _wgc.Stop();
        LiveImage.Visibility = Visibility.Collapsed;
        RouteCanvas.Background = (Brush)FindResource("GridBrush");
    }

    private void LiveTick(object? sender, EventArgs e)
    {
        IntPtr hwnd = SelectedHwnd();
        if (hwnd == IntPtr.Zero) return;

        byte[]? bgra = null;
        int tw = 0, th = 0;
        bool viaWgc = false;

        if (_wgc.IsRunning && _wgc.TryGetLatest(out var full, out int fw, out int fh)
            && fw > 0 && fh > 0 && full.Length >= fw * fh * 4)
        {
            th = 480;
            tw = Math.Clamp((int)Math.Round(th * (double)fw / fh), 160, 1280);
            bgra = DownscaleBgra(full, fw, fh, tw, th);
            viaWgc = true;
        }
        else
        {
            if (!Win32.TryGetClientRectOnScreen(hwnd, out var rect) || rect.Width < 4 || rect.Height < 4) return;
            th = 480;
            tw = Math.Clamp((int)Math.Round(th * (double)rect.Width / rect.Height), 160, 1280);
            bgra = _liveCapture.CaptureBgra(hwnd, tw, th);
        }
        if (bgra == null) return;

        if (_liveWb == null || _liveWb.PixelWidth != tw || _liveWb.PixelHeight != th)
        {
            _liveWb = new WriteableBitmap(tw, th, 96, 96, PixelFormats.Bgra32, null);
            LiveImage.Source = _liveWb;
        }
        _liveWb.WritePixels(new Int32Rect(0, 0, tw, th), bgra, tw * 4, 0);

        // подсказка про перекрытие — только для GDI-фолбэка; WGC ловит окно даже под ботом
        bool dark = !viaWgc && IsDark(bgra);
        if (_liveWasDark != dark)
        {
            _liveWasDark = dark;
            SetStatus(dark
                ? "Кадр чёрный: окно Roblox перекрыто/свёрнуто — отодвинь бот вбок (или WGC не стартовал)."
                : (viaWgc ? "Лайв (WGC): вижу игру." : "Лайв: вижу игру."));
        }
    }

    private static byte[] DownscaleBgra(byte[] src, int sw, int sh, int dw, int dh)
    {
        var dst = new byte[dw * dh * 4];
        for (int y = 0; y < dh; y++)
        {
            int sy = (int)((long)y * sh / dh);
            int srow = sy * sw * 4;
            int drow = y * dw * 4;
            for (int x = 0; x < dw; x++)
            {
                int sx = (int)((long)x * sw / dw);
                int si = srow + sx * 4;
                int di = drow + x * 4;
                dst[di] = src[si];
                dst[di + 1] = src[si + 1];
                dst[di + 2] = src[si + 2];
                dst[di + 3] = 255;
            }
        }
        return dst;
    }

    private static bool IsDark(byte[] bgra)
    {
        long sum = 0; int n = 0;
        for (int i = 0; i < bgra.Length; i += 401) { sum += bgra[i]; n++; }
        return n == 0 || sum / (double)n < 6.0;
    }

    // ===== Рулеры на канвасе =====

    private void RouteCanvas_SizeChanged(object sender, SizeChangedEventArgs e) => DrawRulers();

    private void DrawRulers()
    {
        RulerCanvas.Children.Clear();
        double w = RouteCanvas.ActualWidth, h = RouteCanvas.ActualHeight;
        if (w < 4 || h < 4) return;

        var tick = Brush("RulerTick");
        var lblBrush = Brush("RulerLabel");

        for (int x = 0; x <= w; x += 22)
        {
            bool major = x % 110 == 0;
            RulerCanvas.Children.Add(new Line { X1 = x, Y1 = 0, X2 = x, Y2 = major ? 7 : 4, Stroke = tick, StrokeThickness = 1 });
            if (major && x > 0)
                AddLabel(x + 3, 3, x.ToString(), lblBrush);
        }
        for (int y = 0; y <= h; y += 22)
        {
            bool major = y % 110 == 0;
            RulerCanvas.Children.Add(new Line { X1 = 0, Y1 = y, X2 = major ? 7 : 4, Y2 = y, Stroke = tick, StrokeThickness = 1 });
            if (major && y > 0)
                AddLabel(4, y + 2, y.ToString(), lblBrush);
        }
    }

    private void AddLabel(double x, double y, string text, Brush brush)
    {
        var tb = new TextBlock { Text = text, FontFamily = new FontFamily("Consolas"), FontSize = 9, Foreground = brush };
        Canvas.SetLeft(tb, x);
        Canvas.SetTop(tb, y);
        RulerCanvas.Children.Add(tb);
    }

    private static float ParseFloat(string s, float def) =>
        float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var v) ? v : def;
}
