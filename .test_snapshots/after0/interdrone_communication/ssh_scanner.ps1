# Take network parameter from command line
param(
    [Parameter(Mandatory=$true, Position=0)]
    [string]$Network,

    [Parameter(Mandatory=$true)]
    [string]$Username,

    [string]$Password = "mrrdt",

    [int]$SSHPort = 22
)

Write-Host "Scanning network $Network for SSH servers..." -ForegroundColor Cyan

# Use nmap and filter out error messages, only get scan results
$nmapOutput = & nmap -p $SSHPort --open $Network 2>$null | findstr "Nmap scan report"

# Extract IP addresses
$sshHosts = @()
foreach ($line in $nmapOutput) {
    if ($line -match "(\d+\.\d+\.\d+\.\d+)") {
        $sshHosts += $matches[1]
    }
}

Write-Host "Found $($sshHosts.Count) host(s) with SSH open" -ForegroundColor Green

if ($sshHosts.Count -gt 0) {
    Write-Host "IPs found:"
    $sshHosts | ForEach-Object { Write-Host "  - $_" }
} else {
    Write-Host "No SSH hosts found." -ForegroundColor Yellow
    exit
}

Write-Host ""

# Try to connect to each host
foreach ($ip in $sshHosts) {
    Write-Host "Trying to connect to $ip..." -ForegroundColor Yellow

    # Test connection with plink
    $output = echo y | & plink -ssh -l $Username -pw $Password $ip "hostname" 2>&1

    # Check for successful connection
    if ($LASTEXITCODE -eq 0 -and $output -notmatch "FATAL ERROR" -and $output -notmatch "Access denied") {
        Write-Host "SUCCESS! Connected to $ip" -ForegroundColor Green
        Write-Host "Hostname: $output" -ForegroundColor Cyan
        $ip | Out-File "successful_connection.txt"
        Write-Host "IP saved to successful_connection.txt`n" -ForegroundColor Green

	# Output the SSH command for future use
        Write-Host "To ssh in, use this command:" -ForegroundColor Yellow
        Write-Host "ssh $Username@$ip" -ForegroundColor Green
        Write-Host ""
        break
    } else {
        Write-Host "Failed to connect to $ip`n" -ForegroundColor Red
    }
}

Write-Host "Script completed" -ForegroundColor Cyan
