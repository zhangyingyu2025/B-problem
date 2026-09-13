param([string]$RecordOnly = '')
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
Set-Location -LiteralPath $repoRoot
$pythonExe = 'C:\Python312\python.exe'
if ($RecordOnly) {
    $formalRun = (Resolve-Path -LiteralPath $RecordOnly).Path
    $formalResult = Get-Content -LiteralPath (Join-Path $formalRun 'result.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($formalResult.data_kind -ne 'operator_declared_official_formal') { throw 'Selected run is not a formal test.' }
} else {
    $formalRun = Join-Path $repoRoot ('B3/结果/e13_formal/run-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff'))
    & $pythonExe -B -X utf8 B3/code/run_e13_formal.py --robot-id 202623001400 --formal-ready --output-dir "$formalRun"
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "Run was not completed normally. Logs: $formalRun. Do not restart just to export."
    }
}
$simulatorLogDir = Join-Path $formalRun 'simulator_logs'
New-Item -ItemType Directory -Path $simulatorLogDir -Force | Out-Null
Write-Host "Save the simulator's original log and screenshots here: $simulatorLogDir"
Invoke-Item -LiteralPath $simulatorLogDir
$caseCode = Read-Host 'Paste this run case code from the simulator, then press Enter'
while ([string]::IsNullOrWhiteSpace($caseCode)) {
    $caseCode = Read-Host 'Case code is required; paste the simulator case code'
}
$caseCode = $caseCode.Trim()
$exportStamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
$manifestPath = Join-Path $formalRun ("evidence-manifest-$exportStamp.json")
$manifest = @{runs=@(@{problem='B3';mode='formal';case_code=$caseCode;actions=(Join-Path $formalRun 'actions.jsonl')})}
$manifest | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $manifestPath -Encoding UTF8
$evidenceDir = Join-Path $formalRun ("evidence-$exportStamp")
& $pythonExe -B -X utf8 tools/test_evidence/export.py --manifest "$manifestPath" --output "$evidenceDir" --ledger 'B3/结果/e13_formal/summary'
if ($LASTEXITCODE -ne 0) { throw "Export failed. Logs and case code are saved in $formalRun. Use -RecordOnly to export without rerunning the test." }
Invoke-Item -LiteralPath (Join-Path $repoRoot 'B3/结果/e13_formal/summary')
Invoke-Item -LiteralPath $evidenceDir
