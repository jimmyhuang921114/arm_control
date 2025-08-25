#!/usr/bin/env bash
set -euo pipefail

# === 可調參數 ===
BRANCH="${1:-main}"           # 如果不是 main，呼叫時傳分支名：bash clean_large_and_ignored.sh your-branch
SIZE_LIMIT="${SIZE_LIMIT:-50M}" # 超過這個大小的 blob 會被移除（含歷史）

echo "==> Step 0: 確認 .gitignore 已包含關鍵規則"
# 若已存在就不重覆加入
add_ignore() {
  local rule="$1"
  grep -qxF "$rule" .gitignore || echo "$rule" >> .gitignore
}
add_ignore '# Huggingface cache & models'
add_ignore 'huggingface/'
add_ignore '**/huggingface/'
add_ignore '**/transformers_cache/'
add_ignore '**/chunk-cache/'
add_ignore '**/hub/'
add_ignore ''
git add .gitignore || true

echo "==> Step 1: 從 index 解除追蹤常見大檔／快取（不刪本機）"
# --ignore-unmatch 可避免沒匹配時出錯
git rm -r --cached --ignore-unmatch \
  Grounded_SAM2_ROS2/huggingface/ \
  '**/huggingface/' '**/transformers_cache/' '**/chunk-cache/' '**/hub/' \
  '*.bin' '*.h5' '*.ckpt' '*.pth' '*.onnx' '*.npz' '*.tar' '*.zip' '*.gz' '*.bz2' || true

# 針對你錯誤訊息中提到的超大 blob 路徑做精準解除追蹤（若存在）
git rm -r --cached --ignore-unmatch \
  'Grounded_SAM2_ROS2/huggingface/hub/models--bert-base-uncased/blobs/68d45e234eb4a928074dfd868cead0219ab85354cc53d20e772753c6bb9169d3' || true

if ! git diff --cached --quiet; then
  git commit -m "chore: remove tracked HF model/cache and large archives from index"
fi

echo "==> Step 2: 建立保險備份分支（避免誤操作可回復）"
BACKUP="backup_before_filter_$(date +%Y%m%d_%H%M%S)"
git branch "$BACKUP" || true
echo "    已建立備份分支：$BACKUP"

echo "==> Step 3: 確認 git-filter-repo 是否可用，若無則安裝（需要 Python）"
if ! command -v git-filter-repo >/dev/null 2>&1; then
  echo "    未找到 git-filter-repo，嘗試以 pip 安裝..."
  pip install --user git-filter-repo >/dev/null
  # 有些環境會把它裝在 ~/.local/bin
  export PATH="$HOME/.local/bin:$PATH"
fi
command -v git-filter-repo >/dev/null 2>&1 || { echo "安裝 git-filter-repo 失敗，請手動安裝後重試。"; exit 1; }

echo "==> Step 4: 從整個歷史移除所有 > $SIZE_LIMIT 的 blob"
git filter-repo --force --strip-blobs-bigger-than "$SIZE_LIMIT"

echo "==> Step 5: 檢查是否仍有大於限制的 blob（純檢查）"
git rev-list --objects --all \
 | git cat-file --batch-check='%(objecttype) %(objectname) %(objectsize) %(rest)' \
 | awk '$1=="blob"{print $3, $2, $4}' \
 | sort -nr \
 | head -n 20

echo "==> Step 6: 安全強推（--force-with-lease）"
git push origin "$BRANCH" --force-with-lease

echo "==> 完成！如果遠端仍阻擋，請把上面 Step 5 的清單貼給我，我幫你針對殘留路徑再處理。"
