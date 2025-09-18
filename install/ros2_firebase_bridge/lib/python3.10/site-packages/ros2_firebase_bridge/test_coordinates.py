# test_coordinates.pyの内容を確認・修正
cat > src/ros2_firebase_bridge/ros2_firebase_bridge/test_coordinates.py << 'EOF'
#!/usr/bin/env python3
"""座標変換テスト - パッケージ版"""

from .coordinate_converter import CoordinateConverter

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

def main():
    test_coordinate_conversion()

if __name__ == "__main__":
    main()
EOF