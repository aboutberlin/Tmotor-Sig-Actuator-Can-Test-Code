#!/bin/bash

# Git 初始化脚本
# 此脚本将初始化 Git 仓库并连接到远程仓库

set -e  # 如果任何命令失败，停止执行

REPO_URL="https://github.com/aboutberlin/Tmotor-Sig-Actuator-Can-Test-Code"
CURRENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo "开始初始化 Git 仓库"
echo "=========================================="
echo "当前目录: $CURRENT_DIR"
echo "远程仓库: $REPO_URL"
echo ""

# 检查是否已经初始化
if [ -d ".git" ]; then
    echo "⚠️  Git 仓库已存在"
    read -p "是否要重新初始化? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "退出脚本"
        exit 0
    fi
    echo "删除现有的 .git 文件夹..."
    rm -rf .git
fi

# 1. 初始化本地 Git 仓库
echo "1️⃣  初始化本地 Git 仓库..."
git init

# 2. 配置用户信息（如果还没有全局配置）
echo ""
echo "2️⃣  配置 Git 用户信息..."
if ! git config user.name &> /dev/null; then
    read -p "请输入你的 Git 用户名 (默认: User): " -r USERNAME
    USERNAME=${USERNAME:-User}
    git config user.name "$USERNAME"
fi

if ! git config user.email &> /dev/null; then
    read -p "请输入你的 Git 邮箱 (默认: user@example.com): " -r EMAIL
    EMAIL=${EMAIL:-user@example.com}
    git config user.email "$EMAIL"
fi

# 3. 添加远程仓库
echo ""
echo "3️⃣  添加远程仓库..."
if git config --get remote.origin.url &> /dev/null; then
    echo "远程仓库已存在，更新 URL..."
    git remote set-url origin "$REPO_URL"
else
    git remote add origin "$REPO_URL"
fi

# 4. 创建 .gitignore（如果不存在）
echo ""
echo "4️⃣  检查 .gitignore..."
if [ ! -f ".gitignore" ]; then
    cat > .gitignore << 'EOF'
# IDE
.vscode/
.idea/
*.swp
*.swo
*~
.DS_Store

# Build
build/
dist/
*.o
*.a
*.so
*.exe
*.dll

# Python
__pycache__/
*.py[cod]
*$py.class
*.egg-info/
.env
venv/

# Temporary
*.tmp
*.log
.env.local
EOF
    echo "✅ 已创建 .gitignore"
else
    echo "✅ .gitignore 已存在"
fi

# 5. 添加所有文件到暂存区
echo ""
echo "5️⃣  添加文件到暂存区..."
git add .

# 6. 创建初始提交
echo ""
echo "6️⃣  创建初始提交..."
git commit -m "Initial commit: Motor CAN Test Project setup" || echo "⚠️  没有新文件要提交"

# 7. 拉取远程内容（如果远程仓库有内容）
echo ""
echo "7️⃣  尝试拉取远程内容..."
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
if git pull origin "$CURRENT_BRANCH" 2>/dev/null; then
    echo "✅ 已拉取远程内容"
else
    echo "⚠️  无法拉取（可能远程仓库为空）"
fi

# 显示最终状态
echo ""
echo "=========================================="
echo "✅ Git 初始化完成！"
echo "=========================================="
echo ""
echo "Git 配置信息:"
echo "  用户名: $(git config user.name)"
echo "  邮箱: $(git config user.email)"
echo "  远程仓库: $(git config --get remote.origin.url)"
echo ""
echo "后续操作:"
echo "  1. 编辑你的代码"
echo "  2. 使用 './auto_push.sh' 自动提交和推送"
echo "  3. 或手动使用:"
echo "     git add ."
echo "     git commit -m '你的提交信息'"
echo "     git push origin main"
