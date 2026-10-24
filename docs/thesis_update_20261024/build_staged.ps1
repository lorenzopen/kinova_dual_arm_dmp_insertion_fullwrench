$ErrorActionPreference = 'Stop'
$project = Join-Path $PSScriptRoot 'project'
$latex = 'C:/Users/Lorenzo/AppData/Local/Programs/MiKTeX/miktex/bin/x64/pdflatex.exe'
$bibtex = 'C:/Users/Lorenzo/AppData/Local/Programs/MiKTeX/miktex/bin/x64/bibtex.exe'
Push-Location $project
try {
    New-Item -ItemType Directory -Force build | Out-Null
    $env:BIBINPUTS = $project
    & $latex -interaction=nonstopmode -halt-on-error -no-shell-escape -output-directory=build main.tex *> build/pass1.txt
    if ($LASTEXITCODE -ne 0) {Get-Content build/pass1.txt -Tail 30; throw 'First LaTeX pass failed'}
    & $bibtex build/main *> build/bibtex.txt
    if ($LASTEXITCODE -ne 0) {Get-Content build/bibtex.txt -Tail 30; throw 'BibTeX failed'}
    & $latex -interaction=nonstopmode -halt-on-error -no-shell-escape -output-directory=build main.tex *> build/pass2.txt
    if ($LASTEXITCODE -ne 0) {Get-Content build/pass2.txt -Tail 30; throw 'Second LaTeX pass failed'}
    & $latex -interaction=nonstopmode -halt-on-error -no-shell-escape -output-directory=build main.tex *> build/pass3.txt
    if ($LASTEXITCODE -ne 0) {Get-Content build/pass3.txt -Tail 30; throw 'Final LaTeX pass failed'}
    Get-Content build/pass3.txt -Tail 8
} finally { Pop-Location }
