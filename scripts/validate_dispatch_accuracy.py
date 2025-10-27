#!/usr/bin/env python3
"""
配車精度検証ツール
Web UIからの配車リクエストとROS2での実際の目的地を比較
"""

import sys
import math
import json

class DispatchAccuracyValidator:
    def __init__(self):
        # firebase_config.yamlと同じ設定
        self.origin_lat = 36.55077
        self.origin_lng = 139.92957
        self.scale_factor = 0.01

        # TurtleBot3 Worldのマップ範囲
        self.map_bounds = {
            'x_min': -4.0,
            'x_max': 4.0,
            'y_min': -4.0,
            'y_max': 4.0
        }

    def gps_to_map(self, lat, lng):
        """GPS → Map座標変換（ROS2側と同じロジック）"""
        R = 6371000  # 地球半径(m)

        dlat = math.radians(lat - self.origin_lat)
        dlng = math.radians(lng - self.origin_lng)

        # メートル単位の実距離
        x_raw = dlng * R * math.cos(math.radians(self.origin_lat))
        y_raw = dlat * R

        # スケール適用
        x = x_raw * self.scale_factor
        y = y_raw * self.scale_factor

        return {
            'x': x,
            'y': y,
            'x_raw': x_raw,
            'y_raw': y_raw,
            'in_bounds': self.is_within_bounds(x, y)
        }

    def map_to_gps(self, x, y):
        """Map → GPS座標変換"""
        R = 6371000

        # スケール逆適用
        x_real = x / self.scale_factor
        y_real = y / self.scale_factor

        lat = self.origin_lat + math.degrees(y_real / R)
        lng = self.origin_lng + math.degrees(
            x_real / (R * math.cos(math.radians(self.origin_lat)))
        )

        return {'lat': lat, 'lng': lng}

    def is_within_bounds(self, x, y):
        """マップ範囲内かチェック"""
        return (
            self.map_bounds['x_min'] <= x <= self.map_bounds['x_max'] and
            self.map_bounds['y_min'] <= y <= self.map_bounds['y_max']
        )

    def test_dispatch_coordinates(self):
        """配車座標のテストケース"""
        print("="*70)
        print("配車精度検証ツール")
        print("="*70)
        print()
        print("設定:")
        print(f"  原点GPS: ({self.origin_lat:.6f}, {self.origin_lng:.6f})")
        print(f"  スケール: {self.scale_factor} (GPS 100m = Map 1m)")
        print(f"  マップ範囲: X[{self.map_bounds['x_min']}, {self.map_bounds['x_max']}]")
        print(f"              Y[{self.map_bounds['y_min']}, {self.map_bounds['y_max']}]")
        print()

        # テストケース: Web UIでクリックされそうな座標
        test_cases = [
            # (GPS lat, GPS lng, 説明)
            (36.55077, 139.92957, "原点（マップ中心）"),
            (36.55080, 139.92960, "北東へ少し（約33m）"),
            (36.55100, 139.93000, "北東へ（約285m）"),
            (36.55050, 139.92900, "南西へ（約360m）"),
            (36.55200, 139.93200, "遠い北東（約1.5km）"),
        ]

        print("="*70)
        print("配車座標テスト")
        print("="*70)

        for i, (lat, lng, desc) in enumerate(test_cases, 1):
            print(f"\nテスト {i}: {desc}")
            print(f"  入力GPS: ({lat:.6f}, {lng:.6f})")

            result = self.gps_to_map(lat, lng)

            print(f"  実距離: X={result['x_raw']:.2f}m, Y={result['y_raw']:.2f}m")
            print(f"  Map座標: X={result['x']:.2f}m, Y={result['y']:.2f}m")

            if result['in_bounds']:
                print(f"  ✅ マップ範囲内 - ロボットが到達可能")
            else:
                print(f"  ❌ マップ範囲外 - 自動補正が必要")

                # 補正後の座標を計算
                x_clamped = max(
                    self.map_bounds['x_min'] + 0.5,
                    min(result['x'], self.map_bounds['x_max'] - 0.5)
                )
                y_clamped = max(
                    self.map_bounds['y_min'] + 0.5,
                    min(result['y'], self.map_bounds['y_max'] - 0.5)
                )

                corrected_gps = self.map_to_gps(x_clamped, y_clamped)
                print(f"  🔧 補正Map座標: X={x_clamped:.2f}m, Y={y_clamped:.2f}m")
                print(f"  🔧 補正GPS座標: ({corrected_gps['lat']:.6f}, {corrected_gps['lng']:.6f})")

        print()
        print("="*70)
        print("推奨アクション")
        print("="*70)
        print()
        print("1. 配車テスト手順:")
        print("   a) Web UIを開く (http://localhost:8000)")
        print("   b) 地図上の以下の位置をクリック:")
        print(f"      - 原点付近: {test_cases[0][0]:.6f}, {test_cases[0][1]:.6f}")
        print(f"      - 北東: {test_cases[1][0]:.6f}, {test_cases[1][1]:.6f}")
        print("   c) ROS2のログで実際の目的地を確認:")
        print("      ros2 topic echo /goal_pose --once")
        print()
        print("2. 座標のずれが大きい場合:")
        print("   - firebase_config.yaml の scale_factor を調整")
        print("   - 現在: 0.01 (GPS 100m = Map 1m)")
        print("   - 選択肢: 0.1 (GPS 10m = Map 1m), 1.0 (GPS 1m = Map 1m)")
        print()
        print("3. マップ範囲外エラーが出る場合:")
        print("   - より原点に近い座標でテスト")
        print("   - または map_bounds を拡大（非推奨）")
        print()

    def interactive_test(self):
        """対話形式での座標変換テスト"""
        print()
        print("="*70)
        print("対話形式テスト")
        print("="*70)

        while True:
            print()
            print("1. GPS → Map座標変換")
            print("2. Map → GPS座標変換")
            print("3. 終了")

            choice = input("\n選択 (1-3): ").strip()

            if choice == '1':
                try:
                    lat = float(input("緯度: "))
                    lng = float(input("経度: "))

                    result = self.gps_to_map(lat, lng)

                    print()
                    print(f"入力GPS: ({lat:.6f}, {lng:.6f})")
                    print(f"実距離: X={result['x_raw']:.2f}m, Y={result['y_raw']:.2f}m")
                    print(f"Map座標: X={result['x']:.2f}m, Y={result['y']:.2f}m")
                    print(f"範囲内: {'✅ はい' if result['in_bounds'] else '❌ いいえ'}")

                except ValueError:
                    print("❌ 数値を入力してください")

            elif choice == '2':
                try:
                    x = float(input("X座標 (m): "))
                    y = float(input("Y座標 (m): "))

                    result = self.map_to_gps(x, y)
                    in_bounds = self.is_within_bounds(x, y)

                    print()
                    print(f"Map座標: ({x:.2f}, {y:.2f})")
                    print(f"GPS座標: ({result['lat']:.6f}, {result['lng']:.6f})")
                    print(f"範囲内: {'✅ はい' if in_bounds else '❌ いいえ'}")

                except ValueError:
                    print("❌ 数値を入力してください")

            elif choice == '3':
                print("\n終了します")
                break
            else:
                print("❌ 1-3を入力してください")


def main():
    validator = DispatchAccuracyValidator()

    # 自動テスト
    validator.test_dispatch_coordinates()

    # 対話形式テスト
    try:
        response = input("\n対話形式テストを実行しますか？ (y/n): ")
        if response.lower() == 'y':
            validator.interactive_test()
    except KeyboardInterrupt:
        print("\n\n終了します")


if __name__ == "__main__":
    main()