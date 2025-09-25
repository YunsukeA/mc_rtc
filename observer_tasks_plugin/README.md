# Observer Tasks Plugin

このプラグインは、mc_rtc環境に独立して追加できるObserver-based ImpedanceTaskとObserver-based AdmittanceTaskを提供します。

## 概要

- **ObserverbasedImpedanceTask**: 外部の観測器によって推定された力情報を使用したインピーダンス制御タスク
- **ObserverbasedAdmittanceTask**: 外部の観測器によって推定された力情報を使用したアドミッタンス制御タスク

## 前提条件

- mc_rtc がインストール済みであること
- CMake 3.1 以上
- C++17 対応コンパイラ

## ビルドとインストール

### 1. ソースコードの準備
```bash
# このディレクトリに移動
cd observer_tasks_plugin
```

### 2. ビルド
```bash
mkdir build
cd build
cmake ..
make
```

### 3. インストール
```bash
sudo make install
```

または、ユーザー環境にインストールする場合：
```bash
# ユーザーのmc_rtcプラグインディレクトリにインストール
make install
```

## 使用方法

### 設定ファイルでの使用

プラグインがインストールされると、既存のmc_rtcコントローラでこれらのタスクを使用できます：

#### ObserverbasedImpedanceTask の例
```yaml
tasks:
  - type: ObserverbasedImpedance
    name: right_hand_impedance
    robot: JAXON
    frame: RightGripper  # または surface: RightGripper (deprecated)
    stiffness: 10.0
    weight: 1000.0
    gains:
      spring: [100, 100, 100, 10, 10, 10]
      damper: [10, 10, 10, 1, 1, 1] 
      mass: [1, 1, 1, 0.1, 0.1, 0.1]
      wrench: [1, 1, 1, 1, 1, 1]
    wrench: [0, 0, 0, 0, 0, 10]  # 目標力・トルク
    exportValue:
      exportContactWrench: true
      exportExternalWrench: true
      usingWrench: "Contact"  # "Contact", "External", "Sensor", "None"
```

#### ObserverbasedAdmittanceTask の例
```yaml
tasks:
  - type: ObserverbasedAdmittance
    name: left_hand_admittance
    robot: JAXON
    frame: LeftGripper
    stiffness: 5.0
    weight: 100.0
    Observerbasedadmittance: [0.01, 0.01, 0.01, 0.001, 0.001, 0.001]  # アドミッタンスゲイン
    wrench: [0, 0, 0, 0, 0, -10]  # 目標力・トルク
    exportValue:
      exportContactWrench: true
      exportExternalWrench: false
      usingWrench: "External"
```

## プラグインの特徴

### 独立性
- 既存のmc_rtc環境を変更することなく追加可能
- 他のプロジェクトに依存しない独立したビルド

### 互換性  
- 既存のmc_rtcコントローラと完全互換
- 標準的なmc_rtcタスクと同じインターフェースを提供

### 柔軟性
- 複数の力推定ソース（Contact, External, Sensor）をサポート
- リアルタイムでの設定変更が可能

## トラブルシューティング

### ビルドエラーの場合
1. mc_rtcが正しくインストールされているか確認
2. CMakeのパスが正しく設定されているか確認
3. C++17に対応したコンパイラを使用しているか確認

### ランタイムエラーの場合
1. プラグインが正しくインストールされているか確認
2. 設定ファイルの文法が正しいか確認
3. 指定したロボットフレームが存在するか確認

## 開発者向け情報

### ファイル構造
```
observer_tasks_plugin/
├── CMakeLists.txt
├── README.md
├── include/
│   ├── ObserverbasedImpedanceTask.h
│   └── ObserverbasedAdmittanceTask.h
└── src/
    ├── ObserverbasedImpedanceTask.cpp
    ├── ObserverbasedAdmittanceTask.cpp
    └── plugin.cpp
```

### カスタマイズ
必要に応じて、ヘッダーファイル内のパラメータや動作を変更できます。変更後は再ビルドとインストールが必要です。

## ライセンス

このプラグインはmc_rtcと同じライセンス条件の下で配布されています。