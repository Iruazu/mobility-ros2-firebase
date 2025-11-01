cmake_minimum_required(VERSION 3.8)
project(ros2_firebase_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依存パッケージの検索
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Pythonパッケージのインストール
ament_python_install_package(${PROJECT_NAME})

# 実行ファイルのインストール
install(PROGRAMS
  scripts/firebase_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# 🔽🔽 これを追加 🔽🔽
# Launch, Config, Worlds フォルダをインストール対象に含める
install(DIRECTORY
  launch
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# テスト設定
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
