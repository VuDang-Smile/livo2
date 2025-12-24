#!/bin/bash
# Script để push code với Personal Access Token

# Nếu chưa có tham số, hỏi người dùng
if [ -z "$1" ] || [ -z "$2" ]; then
    echo "═══════════════════════════════════════════════════════════"
    echo "  Git Push với Personal Access Token"
    echo "═══════════════════════════════════════════════════════════"
    echo ""
    
    if [ -z "$1" ]; then
        read -p "Nhập GitHub Username: " USERNAME
    else
        USERNAME=$1
    fi
    
    if [ -z "$2" ]; then
        read -sp "Nhập Personal Access Token (ghp_...): " TOKEN
        echo ""
    else
        TOKEN=$2
    fi
    
    if [ -z "$USERNAME" ] || [ -z "$TOKEN" ]; then
        echo "❌ Username và Token không được để trống!"
        exit 1
    fi
else
    USERNAME=$1
    TOKEN=$2
fi

# Tạo file credentials
echo "https://${USERNAME}:${TOKEN}@github.com" > ~/.git-credentials
chmod 600 ~/.git-credentials

echo "✅ Đã lưu credentials"
echo "🚀 Đang push code..."

# Push code
cd "$(dirname "$0")"
git push origin An/Localization/Intergration-Livo2

if [ $? -eq 0 ]; then
    echo "✅ Push thành công!"
else
    echo "❌ Push thất bại. Kiểm tra lại token và quyền truy cập."
fi

