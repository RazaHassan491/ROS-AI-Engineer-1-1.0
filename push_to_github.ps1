$git = "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\Common7\IDE\CommonExtensions\Microsoft\TeamFoundation\Team Explorer\Git\cmd\git.exe"
$remoteUrl = "https://github.com/RazaHassan491/ROS-AI-Engineer-1-1.0.git"

Write-Host "Initializing Git..."
& $git init

Write-Host "Setting identity..."
& $git config user.email "raza.hassan@example.com"
& $git config user.name "Raza Hassan"

Write-Host "Adding remote..."
& $git remote remove origin 2>$null
& $git remote add origin $remoteUrl

Write-Host "Staging files..."
& $git add .

Write-Host "Committing..."
& $git commit -m "Final ROS 2 Humble optimizations for Windows and camera fixes"

Write-Host "Pushing to GitHub..."
& $git branch -M main
& $git push -u origin main
