# 用于批量修改world文件的圆柱高度

import os
from lxml import etree

# ===== 配置 =====


INPUT_DIR = "/home/balmung/桌面/fourWS_env/src/BARN_Env/source/world_files"          # 原始 world 文件目录
OUTPUT_DIR = "/home/balmung/桌面/fourWS_env/src/BARN_Env/source/world_files"      # 输出目录
TARGET_HEIGHT = "2"                # 圆柱高度

os.makedirs(OUTPUT_DIR, exist_ok=True)

total_cylinders = 0

for i in range(300):
    in_file = os.path.join(INPUT_DIR, f"world_{i}.world")
    out_file = os.path.join(OUTPUT_DIR, f"world_{i}.world")

    if not os.path.exists(in_file):
        print(f"[WARN] File not found: {in_file}")
        continue

    tree = etree.parse(in_file)
    root = tree.getroot()

    count = 0
    for length_tag in root.xpath(".//geometry/cylinder/length"):
        length_tag.text = TARGET_HEIGHT
        count += 1

    tree.write(
        out_file,
        pretty_print=True,
        xml_declaration=True,
        encoding="UTF-8"
    )

    total_cylinders += count
    print(f"[OK] world_{i}.world → modified {count} cylinders")

print(f"\nDone. Total modified cylinders: {total_cylinders}")
