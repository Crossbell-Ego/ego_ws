#!/usr/bin/env python3
"""
詳細分析 imu_filter_madgwick 的重力檢測行為
展示不同加速度值如何影響濾波器的判斷
"""

import numpy as np
import matplotlib.pyplot as plt

def analyze_gravity_detection():
    """
    分析重力檢測的閾值條件
    """
    
    print("=== IMU Filter Madgwick 重力檢測機制分析 ===\n")
    
    # 測試不同的加速度值
    test_cases = [
        ([0.0, 0.0, 0.0], "完全零值 - 觸發 free fall 警告"),
        ([0.1, 0.1, 0.1], "微小值 - 可能觸發 free fall 警告"),
        ([0.0, 0.0, 9.81], "標準重力 Z 軸 - 正常"),
        ([0.0, 9.81, 0.0], "重力 Y 軸 - 正常"),
        ([9.81, 0.0, 0.0], "重力 X 軸 - 正常"),
        ([0.0, 0.0, 1.0], "低重力 - 可能警告"),
        ([6.93, 6.93, 0.0], "傾斜 45° - 正常"),
        ([0.0, 0.0, -9.81], "倒置重力 - 正常"),
    ]
    
    print("1. 重力向量分析:")
    print("   加速度 [ax, ay, az]  |  重力大小  |  狀態")
    print("   " + "="*50)
    
    for accel, description in test_cases:
        magnitude = np.sqrt(sum(x**2 for x in accel))
        gravity_ratio = magnitude / 9.81 if magnitude > 0 else 0
        
        # 判斷是否會觸發 free fall 警告
        # 通常閾值約為 0.1 * 9.81 ≈ 0.98 m/s²
        if magnitude < 0.98:  # 經驗閾值
            status = "⚠️  FREE FALL"
        elif magnitude < 8.0:
            status = "⚠️  低重力"
        else:
            status = "✅ 正常"
        
        print(f"   {str(accel):20} | {magnitude:8.2f} | {status}")
    
    print(f"\n2. Madgwick 演算法特性:")
    print(f"   • 依賴重力向量確定滾轉/俯仰角度")
    print(f"   • 重力大小閾值: ~1.0 m/s² (經驗值)")
    print(f"   • 警告頻率: 每 100ms")
    print(f"   • 不使用磁力計時: 偏航角會漂移")
    
    print(f"\n3. 您的系統狀況:")
    print(f"   • IMU 硬體返回: [0, 0, 0]")
    print(f"   • 重力大小: 0.0 m/s²")
    print(f"   • 觸發條件: magnitude < threshold")
    print(f"   • 結果: 連續 'free fall' 警告")
    
    print(f"\n4. 解決方案:")
    print(f"   ✅ 軟體配置正確 (madgwick 參數)")
    print(f"   ✅ 主題架構完整 (/imu/data_raw → /imu/data)")
    print(f"   ⚠️  硬體問題: IMU 感測器故障")
    print(f"   🔧 建議: 檢查硬體連接或使用模擬數據")

if __name__ == "__main__":
    analyze_gravity_detection()