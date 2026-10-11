<#
.SYNOPSIS
  Add the boards' TLS listener (with a login) to the Mosquitto Windows service.  Run in a PowerShell opened AS ADMINISTRATOR.

.DESCRIPTION
  What it does, in order (-DryRun does none of it and only shows the plan and the new configuration):
    1. backs up mosquitto.conf to mosquitto.conf.bak-<time>
    2. copies ca.pem, server.pem and server.key (from secrets\tls, made by `python -m fleet.certs make`) into <Mosquitto>\ssb\
       and restricts server.key to SYSTEM and Administrators
    3. asks you for the broker password (hidden) and stores it, hashed, with mosquitto_passwd
    4. writes an ACL: that login may read and write only ssb/#
    5. writes the new mosquitto.conf: your existing lines UNCHANGED, plus one marked block with a TLS listener (port 8883, login required)
    6. restarts the mosquitto service. If it does not come back, the backup is restored and the service restarted again
    7. adds a Windows Firewall rule for the port, Private networks only

  Your existing listener on 1883 (anonymous) is not changed. Restarting the service disconnects whatever uses it for a few seconds.

.EXAMPLE
  .\scripts\setup_mosquitto_tls.ps1 -DryRun
  .\scripts\setup_mosquitto_tls.ps1
#>
[CmdletBinding()]
param(
    [string]$MosquittoDir = "C:\Program Files\Mosquitto",
    [string]$CertDir = "",
    [string]$User = "ssb",
    [int]$Port = 8883,
    [switch]$DryRun,
    [switch]$Force
)

$ErrorActionPreference = "Stop"
function Fail([string]$m) { Write-Host ("STOP: " + $m) -ForegroundColor Red; exit 1 }
function Step([string]$m) { Write-Host ("`n== " + $m) -ForegroundColor Cyan }

$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
if (-not $CertDir) { $CertDir = Join-Path $repo "secrets\tls" }
$py = Join-Path $repo ".venv\Scripts\python.exe"
$conf = Join-Path $MosquittoDir "mosquitto.conf"
$dest = Join-Path $MosquittoDir "ssb"
$passwdExe = Join-Path $MosquittoDir "mosquitto_passwd.exe"
$stamp = Get-Date -Format "yyyyMMdd-HHmmss"
$backup = "$conf.bak-$stamp"

Step "Checking"
$isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin -and -not $DryRun) { Fail "this needs a PowerShell run as Administrator (use -DryRun to look without changing anything)." }
foreach ($f in @($py, $conf, $passwdExe)) { if (-not (Test-Path $f)) { Fail ("not found: " + $f) } }
foreach ($n in @("ca.pem", "server.pem", "server.key")) {
    if (-not (Test-Path (Join-Path $CertDir $n))) { Fail ("missing " + (Join-Path $CertDir $n) + " - run: .\.venv\Scripts\python.exe -m fleet.certs make --host <this PC's address>") }
}
$svc = Get-Service -Name mosquitto -ErrorAction SilentlyContinue
if (-not $svc) { Fail "no Windows service named 'mosquitto' found." }
Write-Host ("service mosquitto: " + $svc.Status + ", config " + $conf)

Push-Location $repo
try {
    $newConf = (& $py -m fleet.mosquitto_conf --existing $conf --dir $dest --port $Port) -join "`r`n"
    if ($LASTEXITCODE -ne 0) { Fail "could not build the new configuration (see the message above)." }
    $acl = (& $py -m fleet.mosquitto_conf --dir $dest --print-acl) -join "`r`n"
} finally { Pop-Location }

if ($DryRun) {
    Step "DRY RUN: nothing is changed. The plan"
    Write-Host ("backup        " + $backup)
    Write-Host ("certificates  " + $CertDir + "  ->  " + $dest)
    Write-Host ("login         " + $User + "  (password asked for, hidden; stored hashed in " + (Join-Path $dest "passwd") + ")")
    Write-Host ("firewall      TCP " + $Port + ", Private profile only")
    Step "The new mosquitto.conf would be"
    Write-Host $newConf
    exit 0
}

Write-Host ""
Write-Host "This restarts the Mosquitto service: anything connected to it (port 1883) is disconnected for a few seconds." -ForegroundColor Yellow
if (-not $Force) {
    $answer = Read-Host "Type YES to continue"
    if ($answer -ne "YES") { Fail "cancelled; nothing was changed." }
}

Step "1. Backup"
Copy-Item $conf $backup
Write-Host ("saved " + $backup)

Step "2. Certificates"
New-Item -ItemType Directory -Force -Path $dest | Out-Null
foreach ($n in @("ca.pem", "server.pem", "server.key")) { Copy-Item (Join-Path $CertDir $n) (Join-Path $dest $n) -Force }
& icacls (Join-Path $dest "server.key") /inheritance:r /grant:r "SYSTEM:(R)" "Administrators:(F)" | Out-Null
Write-Host ("copied to " + $dest + "; server.key readable by SYSTEM and Administrators only")

Step "3. Login for the boards (you type the password; it is stored hashed)"
& $passwdExe -c (Join-Path $dest "passwd") $User
if ($LASTEXITCODE -ne 0) { Fail "mosquitto_passwd failed; the configuration was not changed (backup is unused)." }

# mosquitto_passwd strips inherited permissions and leaves the file readable only by the account that ran it; the service runs as SYSTEM
& icacls (Join-Path $dest "passwd") /inheritance:r /grant:r "SYSTEM:(R)" "Administrators:(F)" | Out-Null
Write-Host "passwd readable by SYSTEM and Administrators only"

Step "4. Access rules"
[System.IO.File]::WriteAllText((Join-Path $dest "acl"), $acl, (New-Object System.Text.UTF8Encoding $false))
Write-Host ("login '" + $User + "' may use only ssb/#")

Step "5. Configuration"
[System.IO.File]::WriteAllText($conf, $newConf + "`r`n", (New-Object System.Text.UTF8Encoding $false))
Write-Host "written (your existing lines are unchanged; the new block is marked)"

Step "6. Restart the service"
Restart-Service mosquitto
$ok = $false
for ($i = 0; $i -lt 40; $i++) {
    Start-Sleep -Milliseconds 500
    $svc.Refresh()
    if ($svc.Status -eq "Running" -and (Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue)) { $ok = $true; break }
}
if (-not $ok) {
    Write-Host "The service did not come back with the new configuration. Restoring your backup." -ForegroundColor Red
    Copy-Item $backup $conf -Force
    Restart-Service mosquitto
    Fail ("restored " + $backup + "; the old configuration is running again. Nothing else was changed except the files in " + $dest)
}
Write-Host ("Mosquitto is running and listening on " + $Port)

Step "7. Firewall"
if (-not (Get-NetFirewallRule -DisplayName "ssb-mqtt-tls" -ErrorAction SilentlyContinue)) {
    New-NetFirewallRule -DisplayName "ssb-mqtt-tls" -Direction Inbound -Protocol TCP -LocalPort $Port -Action Allow -Profile Private | Out-Null
    Write-Host ("rule 'ssb-mqtt-tls' added: inbound TCP " + $Port + ", Private networks only")
} else { Write-Host "rule 'ssb-mqtt-tls' already exists" }

Step "Done"
Write-Host ("Backup of your old configuration: " + $backup)
Write-Host "Next (as yourself, not as Administrator): provision board B with  scripts\provision_wifi.py --port COM13"
