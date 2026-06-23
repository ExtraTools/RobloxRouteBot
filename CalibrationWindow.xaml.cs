using System.Globalization;
using System.Numerics;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.Windows.Threading;
using RobloxRouteBot.Vision;

namespace RobloxRouteBot;

/// <summary>
/// Превью того, что «видит» бот: серый кадр + измеренный сдвиг + стрелка движения + confidence.
/// Правит общий с движком OpticalFlowSettings, поэтому калибровка переносится в бой.
/// </summary>
public partial class CalibrationWindow : Window
{
    private readonly OpticalFlowSettings _settings;
    private readonly OpticalFlowProvider _provider;
    private readonly Action<float> _onScaleChanged;
    private readonly DispatcherTimer _timer;

    private WriteableBitmap? _wb;
    private Line? _arrow;
    private Polygon? _head;
    private bool _ready;

    public CalibrationWindow(OpticalFlowSettings settings, Func<IntPtr> hwndSource, Action<float> onScaleChanged)
    {
        InitializeComponent();
        _settings = settings;
        _onScaleChanged = onScaleChanged;
        _provider = new OpticalFlowProvider(hwndSource, settings);

        _timer = new DispatcherTimer(DispatcherPriority.Render) { Interval = TimeSpan.FromMilliseconds(33) };
        _timer.Tick += OnTick;

        LoadFromSettings();
        _ready = true;

        Closed += (_, _) => _timer.Stop();
    }

    private SolidColorBrush Brush(string key) => (SolidColorBrush)FindResource(key);

    private void LoadFromSettings()
    {
        var inv = CultureInfo.InvariantCulture;
        TxtScale.Text = _settings.WorldPerPixel.ToString("0.###", inv);
        TxtRot.Text = _settings.RotationDeg.ToString("0.#", inv);
        TxtPatch.Text = _settings.PatchSize.ToString(inv);
        TxtSearch.Text = _settings.SearchRadius.ToString(inv);
        ChkInvX.IsChecked = _settings.InvertX;
        ChkInvY.IsChecked = _settings.InvertY;
        ChkSwap.IsChecked = _settings.SwapXy;
    }

    private void Setting_Changed(object sender, RoutedEventArgs e)
    {
        if (!_ready) return;
        var inv = CultureInfo.InvariantCulture;

        if (float.TryParse(TxtScale.Text, NumberStyles.Float, inv, out var scale))
        {
            _settings.WorldPerPixel = scale;
            _onScaleChanged(scale);
        }
        if (float.TryParse(TxtRot.Text, NumberStyles.Float, inv, out var rot)) _settings.RotationDeg = rot;
        if (int.TryParse(TxtPatch.Text, NumberStyles.Integer, inv, out var patch) && patch >= 8) _settings.PatchSize = patch;
        if (int.TryParse(TxtSearch.Text, NumberStyles.Integer, inv, out var search) && search >= 2) _settings.SearchRadius = search;

        _settings.InvertX = ChkInvX.IsChecked == true;
        _settings.InvertY = ChkInvY.IsChecked == true;
        _settings.SwapXy = ChkSwap.IsChecked == true;
    }

    private void BtnLive_Click(object sender, RoutedEventArgs e)
    {
        if (_timer.IsEnabled) { _timer.Stop(); BtnLive.Content = "▶ Превью"; }
        else { _timer.Start(); BtnLive.Content = "■ Стоп"; }
    }

    private void BtnClose_Click(object sender, RoutedEventArgs e) => Close();

    private void OnTick(object? sender, EventArgs e)
    {
        var (frame, shift, conf) = _provider.MeasureStep();
        if (frame == null)
        {
            TxtHint.Visibility = Visibility.Visible;
            TxtHint.Text = "Окно игры не найдено или свёрнуто.";
            return;
        }

        TxtHint.Visibility = Visibility.Collapsed;
        RenderFrame(frame);

        var world = _provider.MapShiftToWorld(shift);
        TxtShift.Text = $"{shift.X,5:0.#}, {shift.Y,5:0.#}";
        TxtWorld.Text = $"{world.X,6:0.##}, {world.Y,6:0.##}";
        TxtConf.Text = conf.ToString("0.00");

        UpdateConfidence(conf);
        DrawArrow(world, conf);
    }

    private void UpdateConfidence(float conf)
    {
        conf = Math.Clamp(conf, 0f, 1f);
        double inner = Math.Max(0, ConfTrack.ActualWidth - 2);
        ConfFill.Width = inner * conf;
        string key = conf < 0.35f ? "Danger" : conf < 0.70f ? "Amber" : "Success";
        ConfFill.Fill = Brush(key);
        TxtConf.Foreground = conf <= 0f ? Brush("TextMuted") : Brush(key);
    }

    private void RenderFrame(GrayFrame frame)
    {
        if (_wb == null || _wb.PixelWidth != frame.Width || _wb.PixelHeight != frame.Height)
        {
            _wb = new WriteableBitmap(frame.Width, frame.Height, 96, 96, PixelFormats.Gray8, null);
            ImgPreview.Source = _wb;
        }
        _wb.WritePixels(new Int32Rect(0, 0, frame.Width, frame.Height), frame.Pixels, frame.Width, 0);
    }

    private void DrawArrow(Vector2 v, float conf)
    {
        double w = Overlay.ActualWidth, h = Overlay.ActualHeight;
        if (w < 4 || h < 4) return;

        if (_arrow == null)
        {
            _arrow = new Line { StrokeThickness = 2.5, StrokeStartLineCap = PenLineCap.Round, StrokeEndLineCap = PenLineCap.Round };
            _head = new Polygon();
            Overlay.Children.Add(_arrow);
            Overlay.Children.Add(_head);
        }

        var brush = conf < 0.35f ? Brush("TextSecondary") : Brush("Accent");
        _arrow!.Stroke = brush;
        _head!.Fill = brush;

        const double gain = 8.0;
        double cx = w / 2, cy = h / 2;
        var end = new Vector2((float)cx + v.X * (float)gain, (float)cy + v.Y * (float)gain);

        _arrow.X1 = cx; _arrow.Y1 = cy; _arrow.X2 = end.X; _arrow.Y2 = end.Y;

        float len = (end - new Vector2((float)cx, (float)cy)).Length();
        if (len > 7f)
        {
            var u = Vector2.Normalize(end - new Vector2((float)cx, (float)cy));
            var perp = new Vector2(-u.Y, u.X);
            const float s = 9f;
            var p1 = end;
            var p2 = end - u * s + perp * (s * 0.5f);
            var p3 = end - u * s - perp * (s * 0.5f);
            _head.Points = new PointCollection { new Point(p1.X, p1.Y), new Point(p2.X, p2.Y), new Point(p3.X, p3.Y) };
            _head.Visibility = Visibility.Visible;
        }
        else
        {
            _head.Visibility = Visibility.Collapsed;
        }
    }
}
