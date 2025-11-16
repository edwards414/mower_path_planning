#!/bin/bash

# Apache 2.0 License Header for Python files
read -r -d '' COPYRIGHT_HEADER << 'EOF'
# Copyright 2024 fxrbindi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

EOF

# 查找所有 Python 文件 (排除 build, install, log 目录)
find src -name "*.py" \
  -not -path "*/build/*" \
  -not -path "*/install/*" \
  -not -path "*/log/*" \
  -not -path "*/__pycache__/*" | while read file; do
  
  # 检查文件是否已有 copyright
  if ! grep -q "Copyright" "$file"; then
    echo "Adding copyright to: $file"
    
    # 检查是否有 shebang
    if head -n 1 "$file" | grep -q "^#!"; then
      # 有 shebang: 保留第一行,在第二行插入 copyright
      {
        head -n 1 "$file"
        echo ""
        echo "$COPYRIGHT_HEADER"
        tail -n +2 "$file"
      } > "$file.tmp"
    else
      # 没有 shebang: 直接在开头插入 copyright
      {
        echo "$COPYRIGHT_HEADER"
        cat "$file"
      } > "$file.tmp"
    fi
    
    mv "$file.tmp" "$file"
  else
    echo "Skipping (already has copyright): $file"
  fi
done

echo "Done!"