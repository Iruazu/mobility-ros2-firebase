#!/usr/bin/env python3
"""
座標変換デバッグツール
GPS座標とMap座標の対応を確認
"""

import sys
import math

class CoordinateDebugger:
    def __init__(self, origin_lat=36.55077, origin_lng=139.92957, scale_factor=0.01):
        self.origin_lat = origin_lat
        self.origin_lng = origin_lng
        self.scale_factor = scale_factor

    def gps_to_map(self, lat, lng):
        """GPS → Map座標変換"""
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
            'y_raw': y_raw
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

    def test_conversion(self):
        """座標変換テスト"""
        print("="*60)
        print("🔍 座標変換デバッグツール")
        print("="*60)
        print(f"\n設定:")
        print(f"  原点GPS: ({self.origin_lat:.6f}, {self.origin_lng:.6f})")
        print(f"  スケールファクター: {self.scale_factor}")
        print(f"  マップ範囲: X[-4.0, 4.0], Y[-4.0, 4.0]")

        # テストケース
        test_cases = [
            # (GPS lat, GPS lng, 説明)
            (36.55077, 139.92957, "原点"),
            (36.55080, 139.92960, "わずかに北東"),
            (36.55100, 139.93000, "北東方向"),
            (36.55050, 139.92900, "南西方向"),
        ]

        print("\n" + "="*60)
        print("📍 GPS → Map 変換テスト")
        print("="*60)

        for lat, lng, desc in test_cases:
            result = self.gps_to_map(lat, lng)
            print(f"\n{desc}:")
            print(f"  GPS: ({lat:.6f}, {lng:.6f})")
            print(f"  実距離: ({result['x_raw']:.2f}m, {result['y_raw']:.2f}m)")
            print(f"  Map座標: ({result['x']:.2f}, {result['y']:.2f})")

            # 範囲チェック
            if -4.0 <= result['x'] <= 4.0 and -4.0 <= result['y'] <= 4.0:
                print(f"  ✅ マップ範囲内")
            else:
                print(f"  ⚠️ マップ範囲外！")

        print("\n" + "="*60)
        print("🗺️ Map → GPS 逆変換テスト")
        print("="*60)

        map_test_cases = [
            (0.0, 0.0, "マップ原点"),
            (1.0, 1.0, "北東 1m"),
            (2.0, 0.0, "東 2m"),
            (0.0, 2.0, "北 2m"),
            (-2.0, -2.0, "南西 2m"),
        ]

        for x, y, desc in map_test_cases:
            result = self.map_to_gps(x, y)
            print(f"\n{desc}:")
            print(f"  Map: ({x:.2f}, {y:.2f})")
            print(f"  GPS: ({result['lat']:.6f}, {result['lng']:.6f})")

            # 逆変換確認
            back = self.gps_to_map(result['lat'], result['lng'])
            error_x = abs(back['x'] - x)
            error_y = abs(back['y'] - y)
            print(f"  逆変換誤差: X={error_x:.4f}m, Y={error_y:.4f}m")

            if error_x < 0.01 and error_y < 0.01:
                print(f"  ✅ 変換精度良好")
            else:
                print(f"  ⚠️ 変換誤差が大きい")


def main():
    print("""
╔══════════════════════════════════════════════════════════╗
║        GPS-Map 座標変換デバッグツール                    ║
║  配車精度の問題を診断します                              ║
╚══════════════════════════════════════════════════════════╝
    """)

    # 現在の設定でテスト
    debugger = CoordinateDebugger(
        origin_lat=36.55077,
        origin_lng=139.92957,
        scale_factor=0.01
    )

    debugger.test_conversion()

    # スケールファクター比較
    print("\n" + "="*60)
    print("⚙️ スケールファクター比較")
    print("="*60)

    test_gps = (36.55100, 139.93000)  # 北東方向の座標

    for scale in [0.01, 0.1, 1.0]:
        debugger_test = CoordinateDebugger(
            origin_lat=36.55077,
            origin_lng=139.92957,
            scale_factor=scale
        )
        result = debugger_test.gps_to_map(test_gps[0], test_gps[1])

        print(f"\nスケール {scale}:")
        print(f"  GPS: ({test_gps[0]:.6f}, {test_gps[1]:.6f})")
        print(f"  Map: ({result['x']:.2f}, {result['y']:.2f})")
        print(f"  実距離: ({result['x_raw']:.2f}m, {result['y_raw']:.2f}m)")

        if scale == 0.01:
            print(f"  📝 現在の設定 (GPS 100m = Map 1m)")
        elif scale == 0.1:
            print(f"  📝 10倍 (GPS 10m = Map 1m)")
        elif scale == 1.0:
            print(f"  📝 実距離 (GPS 1m = Map 1m)")

    print("\n" + "="*60)
    print("💡 推奨アクション")
    print("="*60)
    print("""
1. Web UIで配車した座標をメモ
2. RVizでロボットが実際に向かった座標を確認
3. 以下のコマンドで現在の座標を確認:
   ros2 topic echo /odom --once

4. スケールファクターが適切か検証
   - GPS距離100m → Map距離1mなら: scale_factor=0.01 (現在)
   - GPS距離10m → Map距離1mなら: scale_factor=0.1
   - GPS距離1m → Map距離1mなら: scale_factor=1.0
    """)


if __name__ == "__main__":
    main()