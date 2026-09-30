# CI runner：GitHub-hosted + self-hosted 混合模式

`.github/workflows/build.yml` 的每個 job 在哪裡跑，由 repo 變數決定；沒設定就用 GitHub-hosted，和以前一樣。

## 為什麼

repo 是 private，GitHub-hosted runner 的每一分鐘都算進方案額度，而且每個 job 無條件進位到整分鐘。2026-09-20 的 run 149（一個 PR）：

| Job | Runner | 實際時間 | 計費 |
|---|---|---|---|
| Build runtime（arm64 映像） | `ubuntu-24.04-arm` | 11.4 分 | 12 分 |
| mower_hardware tests | `ubuntu-latest` | 1.3 分 | 2 分 |
| STM32 firmware | `ubuntu-latest` | 1.0 分 | 1 分 |
| Pure Python tests | `ubuntu-latest` | 0.3 分 | 1 分 |

每個 run 約 16 分，約 75% 是 arm64 映像。一個 PR 在開 PR、每次 push、merge 到 main 各跑一次。

## 現在的設定

| 變數（Settings → Secrets and variables → Actions → Variables） | 影響的 job | 沒設定時 |
|---|---|---|
| `IMAGE_RUNNER` | Build runtime（arm64 映像） | `ubuntu-24.04-arm` |
| `CI_RUNNER` | Python / mower_coverage_core / firmware / mower_hardware 測試 | `ubuntu-latest` |

值是 JSON 的 label 清單，例如 `["self-hosted", "linux", "arm64"]`。刪掉變數就回到 GitHub-hosted，runner 離線時改一個變數就能退回。

建議：`IMAGE_RUNNER` 指向 self-hosted arm64，`CI_RUNNER` 先不設（測試 job 每個只計 1–2 分）。Actions 額度用完時再把 `CI_RUNNER` 也設成 self-hosted。

不論哪種 runner 都會省的部分：

- 同一個 PR 有新的 push 時，還在跑的舊 run 會被取消（`concurrency`）。main 和 tag 的 run 一定跑完。
- PR 加 label 時只有 `push-image` 會觸發 job，其他 label 不會再整個重跑。
- self-hosted 上不再經過 `actions/cache` / `type=gha` 上傳下載 BuildKit 快取（每次約 2.5 分），快取留在機器上的 `mower-ci` builder 裡。

四個測試 job 都在容器裡跑（`python:3.12-slim`、`rust:1-bookworm`、`ubuntu:24.04`、`ros:jazzy-ros-base`），所以任何裝了 Docker 的 Linux runner 都能跑，不需要 sudo 或 setup-python 的 toolcache。

## 架一台 self-hosted arm64 runner

映像是 arm64（機器人用），所以 runner 要是 arm64 Linux 才能原生建置（x86 上用 QEMU 要 80 分鐘以上）。可以用：

- Apple Silicon Mac 上的 Linux VM（OrbStack、Colima 或 UTM，Ubuntu 24.04 arm64），或
- 一台 arm64 Linux 機器（例如 8 GB 以上的 Raspberry Pi 5 / RK3588 板子，接 SSD）。

**不要把 runner 裝在機器人（LubanCat）上**：它的 CPU 已經很滿，而且 runner 會執行任何分支上的程式碼。

需求：4 核以上、8 GB RAM 以上、至少 60 GB 可用空間（colcon build tree + ccache + cargo + 映像）。

1. 安裝 Docker、git、`git-restore-mtime`，讓 runner 的使用者能用 docker（不需要 sudo）：

   ```bash
   sudo apt-get install -y docker.io docker-buildx git git-restore-mtime
   sudo usermod -aG docker "$USER"   # 重新登入
   ```

2. 建立長期存在的 builder（workflow 在 self-hosted 上固定用這個名字）：

   ```bash
   docker buildx create --name mower-ci --driver docker-container --bootstrap
   ```

3. 註冊 runner：GitHub → repo → Settings → Actions → Runners → New self-hosted runner，選 Linux / ARM64，照頁面上的指令下載並 `./config.sh`，label 保留預設的 `self-hosted, Linux, ARM64`。裝成服務：

   ```bash
   sudo ./svc.sh install && sudo ./svc.sh start
   ```

4. 設 repo 變數 `IMAGE_RUNNER` = `["self-hosted", "linux", "arm64"]`（label 比對不分大小寫）。

5. 推一個 commit 或手動觸發 workflow（Actions → Build Docker Images → Run workflow），確認 `Build runtime` 的 runner 名稱是你的機器。

## 維護

- 快取會一直長：每週清一次，保留最近的部分

  ```bash
  docker buildx prune --builder mower-ci --keep-storage 40gb -f
  docker image prune -f
  ```

- PR 的 smoke test 映像在 job 結束時會自動刪掉；main 的映像推到 GHCR 後留在本機的 tag 可以用 `docker image prune -a` 清。
- `mower-ci` builder 不見了（例如 Docker 重裝）就重跑第 2 步，否則映像 job 會失敗。

## 安全

- runner 會執行 repo 裡任何有權限推分支或開 PR 的人寫的程式碼。repo 是 private、只有協作者，但仍建議讓 runner 在獨立的 VM 裡，不要放 SSH key、雲端憑證或能連到機器人的網路設定。
- 映像 job 在 push / tag / 加了 `push-image` label 的 PR 上會用 `GITHUB_TOKEN`（`packages: write`）推到 GHCR；token 只在該 job 期間有效。

## 注意

- self-hosted runner 不消耗 Actions 分鐘數。但如果帳號是因為付款問題被鎖（2026-09-20 起 main 上每個 job 都在 3 秒內失敗、沒有分到 runner），GitHub 是否仍會派工給 self-hosted runner，要設好之後實際觸發一次確認；帳務問題本身要在 GitHub → Settings → Billing 處理。
- GitHub-hosted 的 `ubuntu-24.04-arm` 在 private repo 的計費倍率以 GitHub 目前的價目表為準。
