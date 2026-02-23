#!/usr/bin/env python3
"""
IMU 飄移問題診斷和解決方案
"""

def diagnose_drift_issues():
    print("=== RVIZ 飄移問題診斷 ===\n")
    
    print("1. 當前問題分析:")
    print("   🔴 IMU 硬體: 返回全零值 [0,0,0]")
    print("   🔴 重力檢測: 無法確定重力方向")
    print("   🔴 姿態估計: 滾轉/俯仰角不穩定")
    print("   🔴 濾波器: 連續 'free fall' 警告")
    
    print("\n2. 參數優化 (已完成):")
    print("   ✅ gain: 0.1 (平衡融合)")
    print("   ✅ constant_dt: 0.01 (穩定時間)")
    print("   ✅ remove_gravity_vector: True")
    print("   ✅ world_frame: 'enu'")
    
    print("\n3. 飄移原因排序:")
    print("   1. 🥇 IMU 硬體故障 (主要原因)")
    print("   2. 🥈 缺少 gain/constant_dt 參數")
    print("   3. 🥉 EKF 融合權重設置")
    print("   4. 4️⃣ 座標系不一致")
    
    print("\n4. 解決方案 (優先級):")
    print("   🔧 立即:")
    print("      - 使用 test_imu_data.py 提供模擬數據")
    print("      - 參數已優化 (gain=0.1, constant_dt=0.01)")
    print("   🔧 短期:")
    print("      - 檢查 IMU 硬體連接")
    print("      - 測試其他 IMU 模組")
    print("   🔧 長期:")
    print("      - 更換可靠的 IMU 感測器")
    print("      - 校準 IMU 偏差參數")
    
    print("\n5. 測試建議:")
    print("   🧪 運行測試腳本:")
    print("      python3 test_imu_data.py")
    print("   🧪 啟動完整系統:")
    print("      ros2 launch egocar_bringup bringup_launch.py")
    print("   🧪 在 RVIZ 中觀察:")
    print("      - 機器人模型是否穩定")
    print("      - TF 樹是否正常")
    print("      - 姿態變化是否合理")

if __name__ == "__main__":
    diagnose_drift_issues()