# PowerShell script to export environment variables for Docker Compose
# Usage: .\export_env.ps1

$ENV_FILE = "../.devcontainer/.env"

# Get current user information
$USER_NAME = $env:USERNAME
$USER_ID = "1000"  # Default UID for Linux containers
$GROUP_NAME = "users"  # Default group for Linux containers
$GROUP_ID = "1000"  # Default GID for Linux containers

# For Windows with WSL2, you might want to use WSL user info
# Uncomment the following lines if you're using WSL2:
# $USER_ID = wsl id -u
# $GROUP_ID = wsl id -g
# $GROUP_NAME = wsl id -gn

# Create .env file
@"
USER_NAME=$USER_NAME
USER_ID=$USER_ID
GROUP_NAME=$GROUP_NAME
GROUP_ID=$GROUP_ID
DISPLAY=host.docker.internal:0
"@ | Out-File -FilePath $ENV_FILE -Encoding ASCII

Write-Host "Environment file created at: $ENV_FILE" -ForegroundColor Green
Write-Host "Contents:" -ForegroundColor Cyan
Get-Content $ENV_FILE
