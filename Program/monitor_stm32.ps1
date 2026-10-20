# OpenOCD Memory Monitor - Read NUC_Obstacle variables
$host.UI.RawUI.OutputEncoding = [System.Text.Encoding]::UTF8

$ADDR_VALID = 0x2400005C
$ADDR_X     = 0x2400005E
$ADDR_Y     = 0x24000060

Write-Host "Connecting to OpenOCD (localhost:4444)..." -ForegroundColor Cyan

try {
    $client = New-Object System.Net.Sockets.TcpClient("localhost", 4444)
    $stream = $client.GetStream()
    $reader = New-Object System.IO.StreamReader($stream)
    $writer = New-Object System.IO.StreamWriter($stream)
    $writer.AutoFlush = $true

    Start-Sleep -Milliseconds 500
    while ($stream.DataAvailable) {
        $null = $reader.ReadLine()
    }

    Write-Host "Connected to OpenOCD`n" -ForegroundColor Green
    Write-Host "Monitoring NUC_Obstacle data (Ctrl+C to exit)"
    Write-Host ("=" * 60)

    $frameCount = 0
    $lastValid = -1
    $lastX = 99999
    $lastY = 99999

    while ($true) {
        # Read valid byte
        $writer.WriteLine("mdb 0x2400005C")
        Start-Sleep -Milliseconds 50
        $validResp = ""
        while ($stream.DataAvailable) {
            $validResp += $reader.ReadLine() + "`n"
        }

        # Read X (int16)
        $writer.WriteLine("mdh 0x2400005E")
        Start-Sleep -Milliseconds 50
        $xResp = ""
        while ($stream.DataAvailable) {
            $xResp += $reader.ReadLine() + "`n"
        }

        # Read Y (int16)
        $writer.WriteLine("mdh 0x24000060")
        Start-Sleep -Milliseconds 50
        $yResp = ""
        while ($stream.DataAvailable) {
            $yResp += $reader.ReadLine() + "`n"
        }

        # Parse responses
        $valid = 0
        $x = 0
        $y = 0

        if ($validResp -match '0x[0-9a-f]+:\s+([0-9a-f]{2})') {
            $valid = [Convert]::ToInt32($matches[1], 16)
        }

        if ($xResp -match '0x[0-9a-f]+:\s+([0-9a-f]{4})') {
            $xVal = [Convert]::ToInt32($matches[1], 16)
            if ($xVal -ge 0x8000) { $xVal -= 0x10000 }
            $x = $xVal
        }

        if ($yResp -match '0x[0-9a-f]+:\s+([0-9a-f]{4})') {
            $yVal = [Convert]::ToInt32($matches[1], 16)
            if ($yVal -ge 0x8000) { $yVal -= 0x10000 }
            $y = $yVal
        }

        # Print only when changed
        if ($valid -ne $lastValid -or $x -ne $lastX -or $y -ne $lastY) {
            $timestamp = Get-Date -Format "HH:mm:ss"

            if ($valid -eq 1) {
                $frameCount++
                $xStr = $x.ToString().PadLeft(5)
                $yStr = if ($y -ge 0) { "+$y" } else { "$y" }
                Write-Host "[$timestamp] [$($frameCount.ToString().PadLeft(4))] [VALID]   X=${xStr}mm  Y=${yStr}mm" -ForegroundColor Green
            } else {
                Write-Host "[$timestamp] [TIMEOUT] (>200ms no data)" -ForegroundColor Yellow
            }

            $lastValid = $valid
            $lastX = $x
            $lastY = $y
        }

        Start-Sleep -Milliseconds 100
    }
}
catch {
    Write-Host "`nError: $_" -ForegroundColor Red
}
finally {
    if ($client) { $client.Close() }
    Write-Host "`nMonitoring stopped. Total frames: $frameCount"
}
