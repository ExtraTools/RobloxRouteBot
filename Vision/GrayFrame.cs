namespace RobloxRouteBot.Vision;

/// <summary>Одноканальный (серый) кадр, top-down, pixels.Length == Width*Height.</summary>
public sealed class GrayFrame
{
    public int Width { get; }
    public int Height { get; }
    public byte[] Pixels { get; }

    public GrayFrame(int width, int height)
    {
        Width = width;
        Height = height;
        Pixels = new byte[width * height];
    }

    public byte At(int x, int y) => Pixels[y * Width + x];
}
