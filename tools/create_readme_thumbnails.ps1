param(
    [int]$CanvasWidth = 960,
    [int]$CanvasHeight = 600
)

$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing

$repositoryRoot = Split-Path -Parent $PSScriptRoot
$sourceDirectory = Join-Path $repositoryRoot 'aubo\video_or_img'
$outputDirectory = Join-Path $sourceDirectory 'readme'
New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null

$background = [System.Drawing.Color]::FromArgb(248, 249, 250)
$sourceFiles = Get-ChildItem -LiteralPath $sourceDirectory -Filter '*.png' -File

foreach ($sourceFile in $sourceFiles) {
    $sourceImage = [System.Drawing.Image]::FromFile($sourceFile.FullName)
    $canvas = New-Object System.Drawing.Bitmap($CanvasWidth, $CanvasHeight)
    $graphics = [System.Drawing.Graphics]::FromImage($canvas)

    try {
        $graphics.Clear($background)
        $graphics.CompositingQuality = [System.Drawing.Drawing2D.CompositingQuality]::HighQuality
        $graphics.InterpolationMode = [System.Drawing.Drawing2D.InterpolationMode]::HighQualityBicubic
        $graphics.PixelOffsetMode = [System.Drawing.Drawing2D.PixelOffsetMode]::HighQuality
        $graphics.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::HighQuality

        $scale = [Math]::Min(
            $CanvasWidth / [double]$sourceImage.Width,
            $CanvasHeight / [double]$sourceImage.Height
        )
        $targetWidth = [int][Math]::Round($sourceImage.Width * $scale)
        $targetHeight = [int][Math]::Round($sourceImage.Height * $scale)
        $targetX = [int][Math]::Floor(($CanvasWidth - $targetWidth) / 2.0)
        $targetY = [int][Math]::Floor(($CanvasHeight - $targetHeight) / 2.0)

        $targetRectangle = New-Object System.Drawing.Rectangle(
            $targetX,
            $targetY,
            $targetWidth,
            $targetHeight
        )
        $graphics.DrawImage($sourceImage, $targetRectangle)

        $outputPath = Join-Path $outputDirectory $sourceFile.Name
        $canvas.Save($outputPath, [System.Drawing.Imaging.ImageFormat]::Png)
        Write-Output "Generated $outputPath"
    }
    finally {
        $graphics.Dispose()
        $canvas.Dispose()
        $sourceImage.Dispose()
    }
}
