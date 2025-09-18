#!/usr/bin/env python3
"""座標変換テスト - パッケージ版"""

from .coordinate_converter import CoordinateConverter

def main():
    test_coordinate_conversion()

# 既存のtest_coordinate_conversion関数はそのまま#!/usr/bin/env python3
"""座標変換テスト"""

import math

class CoordinateConverter:
    def __init__(self, origin_lat=36.5598, origin_lng=139.9088):
        self.origin_lat = origin_lat
        self.origin_lng = origin_lng
        print(f"座標系原点: ({origin_lat}, {origin_lng})")

    def gps_to_map_coordinates(self, lat, lng):
        """GPS座標をROS2マップ座標系に変換"""
        R = 6371000  # 地球半径(m)

        dlat = math.radians(lat - self.origin_lat)
        dlng = math.radians(lng - self.origin_lng)

        x = dlng * R * math.cos(math.radians(self.origin_lat))
        y = dlat * R

        return {'x': x, 'y': y}

    def map_to_gps_coordinates(self, x, y):
        """ROS2マップ座標をGPS座標に変換"""
        R = 6371000

        lat = self.origin_lat + math.degrees(y / R)
        lng = self.origin_lng + math.degrees(x / (R * math.cos(math.radians(self.origin_lat))))

        return {'lat': lat, 'lng': lng}

def test_coordinate_conversion():
    print("🧭 座標変換テストを開始します...\n")

    converter = CoordinateConverter()

    # テストポイント（キャンパス周辺の座標）
    test_points = [
        {"name": "原点", "lat": 36.5598, "lng": 139.9088},
        {"name": "北に100m", "lat": 36.5607, "lng": 139.9088},
        {"name": "東に100m", "lat": 36.5598, "lng": 139.9101},
        {"name": "南東に200m", "lat": 36.5580, "lng": 139.9110},
    ]

    print("=== GPS → マップ座標 変換テスト ===")
    for point in test_points:
        map_coords = converter.gps_to_map_coordinates(point["lat"], point["lng"])
        print(f"{point['name']}:")
        print(f"  GPS: ({point['lat']:.6f}, {point['lng']:.6f})")
        print(f"  Map: ({map_coords['x']:.2f}m, {map_coords['y']:.2f}m)")
        print()

    print("=== マップ座標 → GPS 逆変換テスト ===")
    test_map_points = [
        {"name": "原点", "x": 0.0, "y": 0.0},
        {"name": "北に50m", "x": 0.0, "y": 50.0},
        {"name": "東に50m", "x": 50.0, "y": 0.0},
        {"name": "南西に100m", "x": -100.0, "y": -100.0},
    ]

    for point in test_map_points:
        gps_coords = converter.map_to_gps_coordinates(point["x"], point["y"])
        print(f"{point['name']}:")
        print(f"  Map: ({point['x']:.1f}m, {point['y']:.1f}m)")
        print(f"  GPS: ({gps_coords['lat']:.6f}, {gps_coords['lng']:.6f})")
        print()

    print("✅ 座標変換テスト完了")

if __name__ == "__main__":
    test_coordinate_conversion()