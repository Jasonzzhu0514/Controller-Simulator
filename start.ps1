$ErrorActionPreference = "Stop"

$RootDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$WebDir = Join-Path $RootDir "web"
Set-Location $WebDir

function Get-PythonLaunch {
    if (Get-Command py -ErrorAction SilentlyContinue) {
        return @{
            FileName = "py"
            PrefixArgs = @("-3")
        }
    }

    if (Get-Command python3 -ErrorAction SilentlyContinue) {
        return @{
            FileName = "python3"
            PrefixArgs = @()
        }
    }

    if (Get-Command python -ErrorAction SilentlyContinue) {
        return @{
            FileName = "python"
            PrefixArgs = @()
        }
    }

    throw "Error: Python is required. Install Python 3, then run start.bat again."
}

function Test-PortAvailable([int]$Port) {
    $listener = $null
    try {
        $address = [System.Net.IPAddress]::Parse("127.0.0.1")
        $listener = [System.Net.Sockets.TcpListener]::new($address, $Port)
        $listener.Start()
        return $true
    } catch {
        return $false
    } finally {
        if ($null -ne $listener) {
            $listener.Stop()
        }
    }
}

$StartPort = 8008
if ($env:PORT) {
    $StartPort = [int]$env:PORT
}

$Port = $null
for ($Candidate = $StartPort; $Candidate -lt $StartPort + 100; $Candidate++) {
    if (Test-PortAvailable $Candidate) {
        $Port = $Candidate
        break
    }
}

if ($null -eq $Port) {
    throw "No free port found from $StartPort to $($StartPort + 99)."
}

$Python = Get-PythonLaunch
$ServerArgs = @()
$ServerArgs += $Python.PrefixArgs
$ServerArgs += @("-m", "http.server", [string]$Port, "--bind", "127.0.0.1")

$Server = Start-Process `
    -FilePath $Python.FileName `
    -ArgumentList $ServerArgs `
    -WorkingDirectory $WebDir `
    -WindowStyle Hidden `
    -PassThru
$Url = "http://127.0.0.1:$Port/index.html"

try {
    Start-Sleep -Milliseconds 400
    if ($Server.HasExited) {
        throw "HTTP server exited immediately."
    }

    Start-Process $Url
    Write-Host "Controller Simulator is running at $Url"
    Write-Host "Press Ctrl+C to stop."

    while (-not $Server.WaitForExit(500)) {
    }
} finally {
    if ($null -ne $Server -and -not $Server.HasExited) {
        $Server.Kill()
        $Server.WaitForExit()
    }
}
