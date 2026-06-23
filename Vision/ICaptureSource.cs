namespace RobloxRouteBot.Vision;

/// <summary>
/// Источник кадров для «зрения». Абстрагирует WGC (работает, даже когда бот сверху) и
/// GDI-фолбэк, чтобы и движок, и лайв-превью брали кадр из ОДНОГО места.
/// </summary>
public interface ICaptureSource
{
    /// <summary>Текущий способ захвата: "WGC" | "GDI" | "—".</summary>
    string Mode { get; }

    /// <summary>
    /// Серый кадр targetW×targetH для анализа. isDuplicate=true, если кадр идентичен предыдущему
    /// (WGC под нагрузкой повторяет кадры — повтор читается как нулевое движение и портит интеграцию).
    /// Возвращает null, если окно недоступно.
    /// </summary>
    GrayFrame? GetGray(int targetW, int targetH, out bool isDuplicate);

    /// <summary>Полный цветной кадр (BGRA, top-down) для превью. false, если недоступно.</summary>
    bool TryGetBgra(out byte[] bgra, out int width, out int height);
}
