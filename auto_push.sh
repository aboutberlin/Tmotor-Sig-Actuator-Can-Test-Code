#!/bin/bash

# 自动推送脚本
# 此脚本会自动提交所有更改并推送到远程仓库

set -e  # 如果任何命令失败，停止执行

echo "=========================================="
echo "开始自动提交和推送"
echo "=========================================="
echo ""

# 检查 Git 仓库
if [ ! -d ".git" ]; then
    echo "❌ 错误: 当前目录不是 Git 仓库"
    echo "请先运行 './setup.sh' 来初始化 Git"
    exit 1
fi

# 检查是否有未提交的更改
if git diff-index --quiet HEAD --; then
    echo "⚠️  没有更改需要提交"
    exit 0
fi

# 显示更改内容
echo "📝 检测到的更改:"
git status --short
echo ""

# 获取提交信息
read -p "请输入提交信息 (默认: Update $(date +%Y-%m-%d)): " -r COMMIT_MSG
COMMIT_MSG=${COMMIT_MSG:-Update $(date +%Y-%m-%d)}

echo ""
echo "1️⃣  添加所有文件到暂存区..."
git add .

echo "2️⃣  创建提交..."
git commit -m "$COMMIT_MSG"

echo ""
echo "3️⃣  推送到远程仓库..."
# 尝试推送到 main 分支，如果失败则尝试 master
if git push -u origin main 2>/dev/null; then
    echo "✅ 已推送到 origin/main"
elif git push -u origin master 2>/dev/null; then
    echo "✅ 已推送到 origin/master"
else
    echo "❌ 推送失败："
    echo "   可能原因:"
    echo "   1. 没有网络连接"
    echo "   2. 远程仓库地址不正确"
    echo "   3. 没有访问权限"
    echo ""
    echo "   请检查后重试："
    echo "   git push -u origin main"
    exit 1
fi

echo ""
echo "=========================================="
echo "✅ 提交和推送完成！"
echo "=========================================="
echo ""
echo "提交信息: $COMMIT_MSG"
echo ""
echo "查看日志/"
git log --oneline -3
