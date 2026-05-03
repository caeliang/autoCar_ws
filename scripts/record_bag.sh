#!/bin/bash
# ROS2 Bag Kaydı Scripti
# Sensör fusyon verilerini kaydet

set -e

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_NAME="my_fusion_bag_${TIMESTAMP}"
BAG_PATH="/home/ranim/autoCar_ws/bags/${BAG_NAME}"

echo "🎥 ROS2 Bag Kaydı Başlanıyor"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📂 Bag Adı: $BAG_NAME"
echo "📂 Kayıt Yolu: $BAG_PATH"
echo ""
echo "🎯 Kaydedilecek Topics:"
echo "   • /prius/odom (Raw Odometry)"
echo "   • /odometry/filtered (EKF Filtreli - Fused)"
echo "   • /prius/imu (IMU Sensörü)"
echo ""
echo "⏳ Kaydı başlatmak için ENTER'a basın..."
echo "⏱️  Kaydı durdurmak için CTRL+C'ye basın"
read

# Bag dosyasını kaydet
ros2 bag record \
    /prius/odom \
    /odometry/filtered \
    /prius/imu \
    -o "$BAG_PATH"

echo ""
echo "✅ Bag kaydı tamamlandı!"
echo "📊 Kaydedilen lokasyon: $BAG_PATH"
