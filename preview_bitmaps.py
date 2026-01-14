#!/usr/bin/env python3
"""
OpenFIRE Logo 位图预览工具
将C数组格式的位图数据转换为可视化的PNG图像
"""

import numpy as np
from PIL import Image
import re

def parse_c_array(c_code):
    """解析C数组字符串，提取数值"""
    # 移除注释和多余空格
    c_code = re.sub(r'//.*$', '', c_code, flags=re.MULTILINE)
    c_code = re.sub(r'/\*.*?\*/', '', c_code, flags=re.DOTALL)
    
    # 提取数组内容
    match = re.search(r'\{(.*?)\}', c_code, re.DOTALL)
    if not match:
        return []
    
    # 分割数字
    numbers = re.findall(r'0x[0-9a-fA-F]+|\d+', match.group(1))
    return [int(num, 16) if num.startswith('0x') else int(num) for num in numbers]

def bitmap_to_image(bitmap_data, width, height):
    """将位图数据转换为PIL图像"""
    # 计算每行字节数
    bytes_per_row = (width + 7) // 8
    
    # 创建图像数组
    img_array = np.zeros((height, width), dtype=np.uint8)
    
    for y in range(height):
        for x in range(width):
            byte_index = y * bytes_per_row + (x // 8)
            bit_index = 7 - (x % 8)
            
            if byte_index < len(bitmap_data):
                if bitmap_data[byte_index] & (1 << bit_index):
                    img_array[y, x] = 255  # 白色
                else:
                    img_array[y, x] = 0    # 黑色
    
    return Image.fromarray(img_array, mode='L')

def main():
    # 读取头文件
    try:
        with open('src/OpenFIRE_logo.h', 'r', encoding='utf-8') as f:
            content = f.read()
    except FileNotFoundError:
        print("错误: 找不到 OpenFIRE_logo.h 文件")
        return
    
    # 解析各种位图
    bitmaps = {
        'customSplashBanner': {
            'data': 'customSplashBanner',
            'width': 'CUSTSPLASHBANN_WIDTH',
            'height': 'CUSTSPLASHBANN_HEIGHT'
        },
        'customSplash': {
            'data': 'customSplash',
            'width': 'CUSTSPLASH_WIDTH',
            'height': 'CUSTSPLASH_HEIGHT'
        }
    }
    
    # 提取尺寸定义
    width_defs = {}
    height_defs = {}
    
    for line in content.split('\n'):
        if '#define' in line:
            parts = line.split()
            if len(parts) >= 3:
                if '_WIDTH' in parts[1]:
                    width_defs[parts[1]] = int(parts[2])
                elif '_HEIGHT' in parts[1]:
                    height_defs[parts[1]] = int(parts[2])
    
    print("=== OpenFIRE Logo 位图预览 ===")
    print(f"找到的宽度定义: {width_defs}")
    print(f"找到的高度定义: {height_defs}")
    print()
    
    # 处理每个位图
    for name, info in bitmaps.items():
        print(f"处理 {name}...")
        
        # 获取尺寸
        width = width_defs.get(info['width'], 0)
        height = height_defs.get(info['height'], 0)
        
        if width == 0 or height == 0:
            print(f"  警告: 无法获取 {name} 的尺寸")
            continue
            
        print(f"  尺寸: {width} x {height}")
        
        # 提取数组数据
        pattern = rf'static constexpr uint8_t {info["data"]}\[\] = \{{(.*?)\}};'
        match = re.search(pattern, content, re.DOTALL)
        
        if not match:
            print(f"  警告: 未找到 {name} 的数据")
            continue
            
        # 解析数组
        array_str = match.group(1)
        # 简单解析 - 按逗号分割
        values = []
        for item in array_str.split(','):
            item = item.strip()
            if item.startswith('0x'):
                values.append(int(item, 16))
            elif item.isdigit():
                values.append(int(item))
        
        print(f"  数据点数: {len(values)}")
        
        # 转换为图像
        try:
            img = bitmap_to_image(values, width, height)
            filename = f"{name}.png"
            img.save(filename)
            print(f"  已保存为: {filename}")
        except Exception as e:
            print(f"  错误: {e}")

if __name__ == "__main__":
    main()