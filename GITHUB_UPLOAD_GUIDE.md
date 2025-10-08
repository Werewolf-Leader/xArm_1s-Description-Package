# GitHub Upload Guide

## Step-by-Step Instructions to Upload Your Project to GitHub

### Step 1: Create GitHub Repository

1. Go to [GitHub.com](https://github.com) and log in
2. Click the **"+"** icon in the top right → **"New repository"**
3. Fill in the details:
   - **Repository name**: `xArm_1s-ROS2-URDF-package` (or `xArm_1s_ROS2_URDF_package`)
   - **Description**: "A professional ROS2 package for XArm 1s robotic manipulator with full simulation and control support"
   - **Visibility**: Public (or Private if you prefer)
   - **DO NOT** initialize with README, .gitignore, or license (we already have these)
4. Click **"Create repository"**

### Step 2: Initialize Git in Your Workspace

Open a terminal and run:

```bash
cd ~/xarm1s_ws

# Initialize git repository
git init

# Add all files
git add .

# Make initial commit
git commit -m "Initial commit: ROS2 Humble/Jazzy compatible XArm package with ros2_control"
```

### Step 3: Connect to GitHub Repository

Replace `YOUR_USERNAME` with your GitHub username:

```bash
# Add remote repository (use the URL GitHub shows you after creating the repo)
git remote add origin https://github.com/YOUR_USERNAME/xArm_1s-ROS2-URDF-package.git

# Or if you see a different URL on GitHub, use that instead
```

### Step 4: Push to GitHub

```bash
# Rename branch to main (if needed)
git branch -M main

# Push to GitHub
git push -u origin main
```

### Step 5: Verify Upload

1. Go to your repository on GitHub
2. Refresh the page
3. You should see all your files uploaded!

---

## Alternative: Using GitHub Desktop

If you prefer a GUI:

1. Download [GitHub Desktop](https://desktop.github.com/)
2. Install and sign in with your GitHub account
3. Click **"Add"** → **"Add existing repository"**
4. Browse to `/home/zephyrus/xarm1s_ws`
5. Click **"Publish repository"**
6. Choose repository name: `xArm_1s-ROS2-URDF-package`
7. Click **"Publish Repository"**

---

## Troubleshooting

### Authentication Required

If Git asks for credentials:

**Option 1: Use Personal Access Token (Recommended)**
1. Go to GitHub → Settings → Developer settings → Personal access tokens → Tokens (classic)
2. Generate new token with `repo` scope
3. Use the token as your password when prompted

**Option 2: Use SSH Key**
```bash
# Generate SSH key
ssh-keygen -t ed25519 -C "your_email@example.com"

# Add to SSH agent
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519

# Copy public key
cat ~/.ssh/id_ed25519.pub

# Add to GitHub: Settings → SSH and GPG keys → New SSH key
# Then use SSH URL: git@github.com:YOUR_USERNAME/xArm_1s-ROS2-URDF-package.git
```

### Large Files Warning

If you get warnings about large files (mesh files):
```bash
# Install Git LFS (Large File Storage)
sudo apt install git-lfs
git lfs install

# Track large files
git lfs track "*.stl"
git add .gitattributes
git commit -m "Add Git LFS for mesh files"
git push
```

---

## What Gets Uploaded

✅ **Included:**
- Source code (`src/xarm_description/`)
- Launch files
- URDF/XACRO files
- Configuration files
- Documentation (README, LICENSE, etc.)
- Helper scripts

❌ **Excluded** (via .gitignore):
- `build/` directory
- `install/` directory
- `log/` directory
- IDE files (`.vscode/`, `.idea/`)
- Compiled files

---

## After Upload

1. **Add a README badge** - GitHub will show your README.md
2. **Create releases** - Tag versions of your software
3. **Enable GitHub Actions** - Set up CI/CD (optional)
4. **Add topics** - Help others find your repo (ros2, robotics, urdf, gazebo)
5. **Share** - Share your repository link!

---

## Quick Commands Reference

```bash
# Check status
git status

# Add changes
git add .

# Commit changes
git commit -m "Your commit message"

# Push changes
git push

# Pull latest changes
git pull

# View commit history
git log --oneline
```

---

Need help? Check [GitHub Docs](https://docs.github.com/) or open an issue!
