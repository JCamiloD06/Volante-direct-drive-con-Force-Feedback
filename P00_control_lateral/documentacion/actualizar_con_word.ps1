# Actualiza la tabla de contenido de los .docx con Word y exporta un PDF de revisión.
# Código auxiliar del registro del software. Uso, powershell -File actualizar_con_word.ps1 [carpeta_pdf]
param([string]$carpetaPdf = "")
$dir = Split-Path -Parent $MyInvocation.MyCommand.Path
$w = New-Object -ComObject Word.Application
$w.DisplayAlerts = 0
try {
    foreach ($f in Get-ChildItem (Join-Path $dir "*.docx") | Where-Object { $_.Name -notlike "~$*" }) {
        $d = $w.Documents.Open($f.FullName, $false, $false, $false)
        for ($i = 1; $i -le $d.TablesOfContents.Count; $i++) { $d.TablesOfContents.Item($i).Update() }
        $d.Save()
        if ($carpetaPdf -ne "") { $d.ExportAsFixedFormat((Join-Path $carpetaPdf ($f.BaseName + ".pdf")), 17) }
        Write-Output "$($f.Name) actualizado"
        $d.Close(0)
    }
} finally { $w.Quit() }
