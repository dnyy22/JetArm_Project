# 焊接頭到舊 IK TCP 的補償

`execute_robot_waypoints.py` 的 CSV 座標現在一律代表**焊接尖端**。送入既有 `.so` IK 服務前，程式會把每個目標轉成它原本的 TCP 座標；`.so` 本身不需、更不應修改。

## 目前描述檔得到的座標

在 `soldering_tool.urdf.xacro`：

```text
T_gripper_tool = Trans(-0.0014, 0, 0.0585) Rz(0) Ry(-pi/2) Rx(pi/2)
T_tool_tip     = Trans(0.10325, 0, 0)
```

所以焊接尖端在 `gripper_servo_link` 座標是：

```text
p_gripper_tip = (-0.0014, 0, 0.16175) m
```

原廠 IK 的 `transform.py` 以 `link3 + tool_link` 定義舊 TCP 長度：

```text
p_gripper_old = (0, 0, 0.05445583202 + 0.112)
              = (0, 0, 0.16645583202) m
```

依原廠模型零 pitch 時 TCP 軸與 `gripper_servo_link` 對齊的約定，預設固定補償（舊 TCP 到焊接尖端）為：

```text
p_old_tip = (-0.0014, 0, -0.00470583202) m
```

對每個焊接尖端目標 `p_W_tip`，且 IK pitch 為 `q`，實際送給舊 IK 的位置為：

```text
p_W_old = p_W_tip - Ry(q) p_old_tip
```

這正是 `tool_frame_adapter.py` 的 `tip_to_legacy()`。平滑軌跡與逐點軌跡都走同一個轉換；反向讀取目前姿態時則用相反轉換，因此起始抬升仍以焊接尖端為準。

## 已確認與未確認項目

`soldering_tool.urdf.xacro.before_guide_tube_test` 與目前版的固定 joint、焊頭 TCP 都相同；差異只有導管 visual 是否顯示，沒有運動學差異。

舊 TCP 的預設來自原廠 Python 運動學常數，並非對黑盒 `.so` 的實測。第一次實機前，請以已知治具點做低速 plan-only 驗證；若實測舊 TCP 原點不同，以 `--legacy-tcp-gripper-xyz X Y Z` 覆寫。實際移動另外要求 `--confirm-tool-transform`。
