param(
    [string]$TwenBlockRoot = "D:\twenblock",
    [switch]$ValidateOnly,
    [switch]$FlashNow,
    [string]$OutputFirmware = ""
)

$ErrorActionPreference = "Stop"

function Get-WavDurationSeconds {
    param([Parameter(Mandatory = $true)][string]$Path)

    $stream = [System.IO.File]::OpenRead($Path)
    $reader = [System.IO.BinaryReader]::new($stream)
    try {
        if ([System.Text.Encoding]::ASCII.GetString($reader.ReadBytes(4)) -ne "RIFF") {
            throw "Not a RIFF WAV file: $Path"
        }
        [void]$reader.ReadUInt32()
        if ([System.Text.Encoding]::ASCII.GetString($reader.ReadBytes(4)) -ne "WAVE") {
            throw "Not a WAVE file: $Path"
        }

        $byteRate = 0
        $dataSize = 0
        while ($stream.Position + 8 -le $stream.Length) {
            $chunkId = [System.Text.Encoding]::ASCII.GetString($reader.ReadBytes(4))
            $chunkSize = $reader.ReadUInt32()
            $chunkStart = $stream.Position

            if ($chunkId -eq "fmt ") {
                if ($chunkSize -lt 12) {
                    throw "Invalid WAV fmt chunk: $Path"
                }
                [void]$reader.ReadUInt16()
                [void]$reader.ReadUInt16()
                [void]$reader.ReadUInt32()
                $byteRate = $reader.ReadUInt32()
            }
            elseif ($chunkId -eq "data") {
                $dataSize = $chunkSize
            }

            $nextChunk = $chunkStart + $chunkSize + ($chunkSize % 2)
            if ($nextChunk -gt $stream.Length) {
                break
            }
            $stream.Position = $nextChunk
        }

        if ($byteRate -le 0 -or $dataSize -le 0) {
            throw "Cannot read WAV duration: $Path"
        }
        return [double]$dataSize / [double]$byteRate
    }
    finally {
        $reader.Dispose()
        $stream.Dispose()
    }
}

$projectDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
$sourceWav = Join-Path $projectDirectory "niulai.wav"
$asrWorkspace = Join-Path $TwenBlockRoot "asrpro"
$lame = Join-Path $asrWorkspace "asr_pro_sdk\tools\lame.exe"
$voiceDirectory = Join-Path $asrWorkspace "voice\mp3"
$makebin = Join-Path $asrWorkspace "makebin.exe"
$rebuildBat = Join-Path $asrWorkspace "rebuild.bat"
$outputMp3 = Join-Path $voiceDirectory "[10001]niulai.mp3"
$firmware = Join-Path $asrWorkspace "fw.bin"
if ([string]::IsNullOrWhiteSpace($OutputFirmware)) {
    $OutputFirmware = Join-Path $projectDirectory "niulaipro_final.bin"
}

foreach ($requiredPath in @($sourceWav, $lame, $voiceDirectory, $makebin, $rebuildBat)) {
    if (-not (Test-Path -LiteralPath $requiredPath)) {
        throw "Required path does not exist: $requiredPath"
    }
}

if ($ValidateOnly) {
    Write-Host "Environment is ready. No files were changed."
    Write-Host "Source WAV: $sourceWav"
    Write-Host "ASR workspace: $asrWorkspace"
    return
}

# Tianwen Block reserves play ID 10001 for the power-on welcome sound.
$sourceDuration = Get-WavDurationSeconds -Path $sourceWav
Write-Host ("[1/4] Source WAV: {0} ({1:F3}s)" -f $sourceWav, $sourceDuration)
Get-ChildItem -LiteralPath $voiceDirectory -File |
    Where-Object { $_.Name.StartsWith("[10001]", [StringComparison]::Ordinal) } |
    ForEach-Object { Remove-Item -LiteralPath $_.FullName -Force }

& $lame --silent --cbr -b16 -t --resample 16000 $sourceWav $outputMp3
if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $outputMp3)) {
    throw "Failed to convert niulai.wav for ASR Pro."
}
Write-Host "[2/4] Converted the complete WAV track: $outputMp3"

$decodedWav = Join-Path ([System.IO.Path]::GetTempPath()) ("niulai_validate_{0}.wav" -f [Guid]::NewGuid().ToString("N"))
try {
    & $lame --silent --decode $outputMp3 $decodedWav
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $decodedWav)) {
        throw "Failed to validate the converted power-on audio."
    }

    $decodedDuration = Get-WavDurationSeconds -Path $decodedWav
    $durationDifference = [Math]::Abs($decodedDuration - $sourceDuration)
    if ($durationDifference -gt 0.25) {
        throw ("Audio duration validation failed: source={0:F3}s converted={1:F3}s" -f $sourceDuration, $decodedDuration)
    }
    Write-Host ("[3/4] Duration validation passed: source={0:F3}s converted={1:F3}s" -f $sourceDuration, $decodedDuration)
}
finally {
    if (Test-Path -LiteralPath $decodedWav) {
        Remove-Item -LiteralPath $decodedWav -Force
    }
}

if ($FlashNow) {
    Write-Host "[4/4] Opening the official ASR Pro flashing tool..."
    Start-Process -FilePath $makebin -WorkingDirectory $asrWorkspace
    Write-Host "Custom audio is ready for flashing."
    Write-Host "Select the ASR Pro download serial port in the flashing window and start flashing."
    Write-Host "Do not compile again before flashing, or Tianwen may overwrite [10001]niulai.mp3."
    return
}

$buildStartedAt = Get-Date
$buildStdout = Join-Path ([System.IO.Path]::GetTempPath()) ("asr_build_{0}.out" -f [Guid]::NewGuid().ToString("N"))
$buildStderr = Join-Path ([System.IO.Path]::GetTempPath()) ("asr_build_{0}.err" -f [Guid]::NewGuid().ToString("N"))
try {
    Write-Host "[4/4] Running the complete Tianwen rebuild pipeline..."
    $buildProcess = Start-Process `
        -FilePath $env:ComSpec `
        -ArgumentList @("/d", "/c", "rebuild.bat") `
        -WorkingDirectory $asrWorkspace `
        -RedirectStandardOutput $buildStdout `
        -RedirectStandardError $buildStderr `
        -NoNewWindow `
        -PassThru

    if (-not $buildProcess.WaitForExit(120000)) {
        & taskkill.exe /PID $buildProcess.Id /T /F | Out-Null
        throw "Tianwen rebuild timed out after 120 seconds."
    }
    $buildProcess.WaitForExit()

    if (Test-Path -LiteralPath $buildStdout) {
        Get-Content -LiteralPath $buildStdout | ForEach-Object { Write-Host $_ }
    }
    if (Test-Path -LiteralPath $buildStderr) {
        Get-Content -LiteralPath $buildStderr | ForEach-Object { Write-Host $_ }
    }
    if ($buildProcess.ExitCode -ne 0) {
        throw "Tianwen rebuild failed with exit code $($buildProcess.ExitCode)."
    }
}
finally {
    Remove-Item -LiteralPath $buildStdout, $buildStderr -Force -ErrorAction SilentlyContinue
}

if (-not (Test-Path -LiteralPath $firmware)) {
    throw "Firmware was not generated: $firmware"
}
$firmwareInfo = Get-Item -LiteralPath $firmware
if ($firmwareInfo.LastWriteTime -lt $buildStartedAt.AddSeconds(-2)) {
    $mp3Info = Get-Item -LiteralPath $outputMp3
    throw ("The rebuild completed but did not update fw.bin. fw.bin={0:yyyy-MM-dd HH:mm:ss}, custom MP3={1:yyyy-MM-dd HH:mm:ss}. Do not flash the old fw.bin." -f $firmwareInfo.LastWriteTime, $mp3Info.LastWriteTime)
}
Write-Host "Tianwen firmware package updated: $firmware"

Copy-Item -LiteralPath $firmware -Destination $OutputFirmware -Force
$firmwareHash = (Get-FileHash -LiteralPath $OutputFirmware -Algorithm SHA256).Hash

Write-Host ""
Write-Host "SUCCESS: the complete niulai.wav track was packaged."
Write-Host ("Duration: source={0:F3}s, packaged={1:F3}s" -f $sourceDuration, $decodedDuration)
Write-Host "Flash this firmware:"
Write-Host $OutputFirmware
Write-Host "SHA256: $firmwareHash"
