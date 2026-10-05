<#
.SYNOPSIS
  Rebuild rpi_capturer for linux/arm64 in Docker and optionally deploy it to the Pi Zero.

.DESCRIPTION
  PowerShell equivalent of build.sh, for Windows with Docker Desktop.
  Exports rpi_capturer/ from the git ref, builds it with the pinned environment
  from the Dockerfile and prints the sha256. For the default ref
  (rpi_capturer/zero-deployed) the result must match expected.sha256.

  -Deploy copies the binary to the Zero over SSH (key auth), keeps the previous
  binary as rpi_capturer.prev when it differs, and verifies the hash remotely.
  A running rpi_capturer is not restarted.

.EXAMPLE
  .\build.ps1
.EXAMPLE
  .\build.ps1 -Deploy
.EXAMPLE
  .\build.ps1 -Ref rpi_capturer/motion-fps-fix -Deploy -ZeroHost pi@192.168.1.147
#>
param(
    [string]$Ref = 'rpi_capturer/zero-deployed',
    [switch]$Deploy,
    [string]$ZeroHost = 'pi@zero.local',
    [string]$RemoteName = 'rpi_capturer'
)

$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot

function Assert-Exit([string]$What) {
    if ($LASTEXITCODE -ne 0) { throw "$What failed (exit code $LASTEXITCODE)" }
}

$DefaultRef = 'rpi_capturer/zero-deployed'
$Tar = "$env:SystemRoot\System32\tar.exe"
$SshOpts = @('-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', '-o', 'StrictHostKeyChecking=accept-new')

# 1. Export sources from git (via a file: piping binary data is unsafe in PowerShell 5.1)
Remove-Item -Recurse -Force src, out -ErrorAction SilentlyContinue
New-Item -ItemType Directory src | Out-Null
$top = (git rev-parse --show-toplevel); Assert-Exit 'git rev-parse'
$srcTar = Join-Path $env:TEMP 'rpi_capturer-src.tar'
git -C $top archive -o $srcTar $Ref rpi_capturer; Assert-Exit "git archive $Ref"
& $Tar -xf $srcTar -C src; Assert-Exit 'tar'
Remove-Item $srcTar

# 2. Build for arm64; only the final 'out' stage is written to .\out
docker buildx build --platform linux/arm64 --target out --output type=local,dest=out .
Assert-Exit 'docker buildx build'

# 3. Check the result
$bin = Join-Path $PSScriptRoot 'out\rpi_capturer'
$hash = (Get-FileHash $bin -Algorithm SHA256).Hash.ToLower()
Write-Host ''
Write-Host "$hash  out/rpi_capturer"
if ($Ref -eq $DefaultRef) {
    $expected = ((Get-Content expected.sha256 -Raw).Trim() -split '\s+')[0]
    if ($hash -ne $expected) { throw "MISMATCH: expected $expected" }
    Write-Host 'OK: bit-identical to the binary deployed on the Pi Zero' -ForegroundColor Green
}

# 4. Deploy
if ($Deploy) {
    Write-Host ''
    Write-Host "Deploying to ${ZeroHost}:~/$RemoteName" -ForegroundColor Cyan
    scp @SshOpts $bin "${ZeroHost}:$RemoteName.new"; Assert-Exit 'scp'
    $remote = "set -e; cd ~; chmod +x $RemoteName.new; " +
              "if [ -f $RemoteName ] && ! cmp -s $RemoteName $RemoteName.new; then cp -p $RemoteName $RemoteName.prev; echo saved previous as $RemoteName.prev; fi; " +
              "mv $RemoteName.new $RemoteName; sha256sum $RemoteName; " +
              "pgrep -a $RemoteName && echo NOTE: running instance keeps the old binary until restarted || true"
    $out = ssh @SshOpts $ZeroHost $remote; Assert-Exit 'ssh'
    $out | ForEach-Object { Write-Host "  zero> $_" }
    $remoteHash = (($out | Select-String -Pattern '^[0-9a-f]{64} ' | Select-Object -First 1).Line -split '\s+')[0]
    if ($remoteHash -ne $hash) { throw "Remote hash $remoteHash does not match $hash" }
    Write-Host 'Deployed and verified.' -ForegroundColor Green
}
