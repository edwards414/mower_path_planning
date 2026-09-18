# Runtime image 瘦身紀錄

`ghcr.io/edwards414/mower_path_planning` (runtime target, arm64)：**4.97 GB → 1.93 GB**（uncompressed；apt 那層 ~4.3 GB → ~1.2 GB）。機器人每次更新只拉有變的層（install tree ~40 MB + 韌體），apt 層只在 package.xml 改變時重抓。

## 4.97 GB 是怎麼來的

runtime 的 apt 清單由 `rosdep keys --dependency-types=exec` 對 `src/` 全部 package 算出（Dockerfile builder 8/8）。三個源頭：

| 來源 | 拉進來的東西 |
|---|---|
| `mower_nav2` 的 `<depend>nav2_bringup</depend>` | Jazzy 的 `nav2_bringup` exec-depends TurtleBot sim → `ros_gz_sim`、gz-sim / dartsim / ogre vendor、VTK、Mesa + 兩份 LLVM、gcc / gfortran / cmake：**約 500 個 deb、2.5 GB** |
| `mower_coverage_core` 的 `<depend>python3</depend>` | rosdep 把 `python3` 對到 `python3-dev`（libpython3.12-dev + headers） |
| ROS deb 本身的 Depends | `-dev` 套件（boost、opencv、icu…）、headers（`/usr/include`、`/opt/ros/jazzy/include` 121 MB）、`/usr/share/doc` 169 MB |

## 做了什麼

1. `mower_nav2/package.xml`：`nav2_bringup` 換成 launch 真正用到的 13 個 nav2 套件（servers + navfn / RPP / rotation shim / simple smoother / costmap_2d / common）。1135 → 646 個新裝 deb。
2. `mower_coverage_core/package.xml`：`python3` 改成 buildtool 依賴。
3. Dockerfile runtime stage（同一個 RUN layer，否則刪掉的檔案還留在上一層）：
   - dpkg `path-exclude` 掉 doc / man / info / locale（保留 copyright）
   - 裝完後 `dpkg --purge --force-depends`：Mesa 軟體 GL + LLVM（`libGL.so` 由 glvnd 保留；headless 不會 render）、ruby（gz-tools-vendor 的 CLI）、sanitizer libs、`libgcc/libstdc++-13-dev`、`libboost-dev`、`proj-data`
   - 刪 `/usr/include`、`/opt/ros/jazzy/include`、cmake 資料與 binary、`*.a`
4. 故意留著：**VTK**（`python3-opencv` 的 `cv2.so` link 到 `libopencv_viz` → 拿掉就 `import cv2` 失敗；mission 的影像遮罩匯入要用）、**boto3/botocore 97 MB**（recorder 上傳 R2）。

## 怎麼驗證的（改 Dockerfile 後照做）

```bash
docker buildx build --target runtime --platform linux/arm64 -t mower-runtime:test --load .
docker run --rm --platform linux/arm64 mower-runtime:test bash -lc '
  set +u; source /opt/ros/jazzy/setup.bash; source /mower_ws/install/setup.bash
  ros2 launch mower_bringup robot.launch.py --show-args >/dev/null
  python3 -c "import cv2, boto3, rclpy, cv_bridge, rosbridge_server, mower_coverage_core"
  for b in /opt/ros/jazzy/lib/{nav2_*,robot_localization,controller_manager,joy}/* /opt/ros/jazzy/lib/libnav2_*.so; do
    ldd "$b" | grep -q "not found" && echo "MISSING deps: $b"; done'
```

PR 的 smoke test（`.github/workflows/build.yml`）也跑同樣的檢查。真機驗證：更新後 `docker logs` 要看到 `Managed nodes are active`，app 連得上。
