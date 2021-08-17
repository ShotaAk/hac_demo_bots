# HAC DEMO Bots

HACのデモパッケージ

## 使い方

次のコマンドを実行すると、
Gazeboフィールド上にラズパイマウスとHACフィールドが表示されます。

ジョイスティックコントローラでラズパイマウスを操作できます。

```sh
$ ros2 launch hac_demo_bots raspimouse.launch.py challenge:=01
```

引数の`challenge`の番号を変更することで、
様々なパターンのフィールドを表示できます。

