# -*- coding: utf-8 -*-
import cv2
import random
import os
from pathlib import Path
import shutil
import numpy as np


class DotGenerator:
    def __init__(self, image_dir, label_dir, output_image_dir=None, output_label_dir=None,
                 min_radius=1, max_radius=10, dot_color=(255, 0, 255), class_id=0,
                 negative_ratio=0.3, overlap_ratio=0.5):
        """
        初始化圆点生成器（支持圆点重叠在其他目标中心）
        
        Args:
            image_dir: 输入图像文件夹路径（原始图像不会被修改）
            label_dir: 原始标注文件夹路径（用于读取原有标注）
            output_image_dir: 输出图像文件夹路径（如果不指定，自动生成）
            output_label_dir: 输出标注文件夹路径（如果不指定，自动生成）
            min_radius: 最小半径(像素)
            max_radius: 最大半径(像素)
            dot_color: 圆点颜色，BGR格式，默认为紫色 (255, 0, 255)
            class_id: 新类别的类别ID
            negative_ratio: 负样本比例，0-1之间，表示多少比例的图片不添加圆点
            overlap_ratio: 圆点落在其他目标中心的比例，0-1之间
        """
        self.image_dir = Path(image_dir)
        self.label_dir = Path(label_dir)
        
        if output_image_dir is None:
            self.output_image_dir = self.image_dir.parent / (self.image_dir.name + "_with_dots")
        else:
            self.output_image_dir = Path(output_image_dir)
        
        if output_label_dir is None:
            self.output_label_dir = self.label_dir.parent / (self.label_dir.name + "_with_dots")
        else:
            self.output_label_dir = Path(output_label_dir)
        
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.dot_color = dot_color
        self.class_id = class_id
        self.negative_ratio = max(0.0, min(1.0, negative_ratio))
        self.overlap_ratio = max(0.0, min(1.0, overlap_ratio))  # 新增：重叠比例
        
        # 创建输出文件夹
        self.output_image_dir.mkdir(parents=True, exist_ok=True)
        self.output_label_dir.mkdir(parents=True, exist_ok=True)
        
        # 支持的图片格式
        self.img_extensions = {'.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.webp', '.JPG', '.PNG'}
        
        # 统计信息
        self.stats = {
            'total': 0,
            'positive': 0,      # 添加了圆点的
            'negative': 0,      # 没有添加圆点的
            'overlap': 0,       # 圆点落在其他目标中心的
            'random': 0,        # 圆点随机放置的
            'failed': 0
        }
    
    def read_existing_labels(self, image_stem):
        """读取已有的标注文件，返回解析后的标注列表"""
        label_path = self.label_dir / f"{image_stem}.txt"
        existing_labels = []
        parsed_labels = []
        
        if label_path.exists():
            with open(label_path, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if line:
                        existing_labels.append(line)
                        parts = line.split()
                        if len(parts) >= 5:
                            try:
                                parsed_labels.append({
                                    'class_id': int(parts[0]),
                                    'x_center': float(parts[1]),
                                    'y_center': float(parts[2]),
                                    'width': float(parts[3]),
                                    'height': float(parts[4]),
                                    'raw': line
                                })
                            except ValueError:
                                pass
        
        return existing_labels, parsed_labels
    
    def parse_yolo_to_pixel(self, yolo_label, img_w, img_h):
        """
        将YOLO标注转换为像素坐标
        返回: (x_min, y_min, x_max, y_max, x_center, y_center)
        """
        parts = yolo_label.split()
        if len(parts) < 5:
            return None
        
        class_id = int(parts[0])
        x_center = float(parts[1]) * img_w
        y_center = float(parts[2]) * img_h
        width = float(parts[3]) * img_w
        height = float(parts[4]) * img_h
        
        x_min = x_center - width / 2
        y_min = y_center - height / 2
        x_max = x_center + width / 2
        y_max = y_center + height / 2
        
        return {
            'class_id': class_id,
            'x_min': x_min,
            'y_min': y_min,
            'x_max': x_max,
            'y_max': y_max,
            'x_center': x_center,
            'y_center': y_center,
            'width': width,
            'height': height
        }
    
    def get_existing_boxes(self, parsed_labels, img_w, img_h):
        """获取已有目标框的像素坐标"""
        boxes = []
        for label in parsed_labels:
            box = self.parse_yolo_to_pixel(label['raw'], img_w, img_h)
            if box:
                boxes.append(box)
        return boxes
    
    def generate_dot_on_target_center(self, img_w, img_h, boxes, radius):
        """
        在随机一个目标框的中心生成圆点
        返回: (x, y) 圆心坐标
        """
        if not boxes:
            return None
        
        # 随机选择一个目标框
        selected_box = random.choice(boxes)
        
        # 使用目标框的中心
        x = int(selected_box['x_center'])
        y = int(selected_box['y_center'])
        
        # 确保圆点在图片范围内
        x = max(radius, min(img_w - radius - 1, x))
        y = max(radius, min(img_h - radius - 1, y))
        
        return x, y
    
    def generate_dot_random(self, img_w, img_h, radius):
        """在图片随机位置生成圆点"""
        x = random.randint(radius, img_w - radius - 1)
        y = random.randint(radius, img_h - radius - 1)
        return x, y
    
    def generate_dot_with_overlap(self, img_w, img_h, boxes, is_overlap):
        """
        根据是否重叠生成圆点位置
        
        Args:
            img_w: 图片宽度
            img_h: 图片高度
            boxes: 已有目标框列表
            is_overlap: 是否使用重叠模式
        
        Returns:
            tuple: (x, y, radius) 或 None
        """
        # 随机生成半径
        radius = random.randint(self.min_radius, self.max_radius)
        radius = max(1, min(radius, min(img_w, img_h) // 4))
        
        if is_overlap and boxes:
            # 重叠模式：选择目标中心
            result = self.generate_dot_on_target_center(img_w, img_h, boxes, radius)
            if result:
                x, y = result
                return x, y, radius
        
        # 随机模式
        x, y = self.generate_dot_random(img_w, img_h, radius)
        return x, y, radius
    
    def generate_dot(self, image_path, parsed_labels, is_negative=False, is_overlap=False):
        """
        在单张图片上生成圆点并返回YOLO标注
        
        Args:
            image_path: 图片路径
            parsed_labels: 已解析的原有标注
            is_negative: 是否为负样本（不添加圆点）
            is_overlap: 是否让圆点落在其他目标中心
        
        Returns:
            tuple: (处理后的图像, YOLO标注或None, 圆点信息或None)
        """
        img = cv2.imread(str(image_path))
        if img is None:
            raise ValueError(f"无法读取图片: {image_path}")
        
        h, w = img.shape[:2]
        
        # 如果是负样本，直接返回原图
        if is_negative:
            return img, None, None
        
        # 获取已有目标框
        boxes = self.get_existing_boxes(parsed_labels, w, h)
        
        # 生成圆点位置
        result = self.generate_dot_with_overlap(w, h, boxes, is_overlap)
        if result is None:
            # 如果无法生成重叠点，回退到随机
            result = self.generate_dot_with_overlap(w, h, [], False)
        
        x, y, radius = result
        
        # 绘制圆点
        cv2.circle(img, (x, y), radius, self.dot_color, -1)
        
        # 计算YOLO格式的边界框
        x_min = x - radius
        y_min = y - radius
        x_max = x + radius
        y_max = y + radius
        
        x_center_norm = (x_min + x_max) / 2 / w
        y_center_norm = (y_min + y_max) / 2 / h
        width_norm = (x_max - x_min) / w
        height_norm = (y_max - y_min) / h
        
        # 确保边界框在[0,1]范围内
        x_center_norm = max(0.0, min(1.0, x_center_norm))
        y_center_norm = max(0.0, min(1.0, y_center_norm))
        width_norm = max(0.0, min(1.0, width_norm))
        height_norm = max(0.0, min(1.0, height_norm))
        
        yolo_label = f"{self.class_id} {x_center_norm:.6f} {y_center_norm:.6f} {width_norm:.6f} {height_norm:.6f}"
        
        return img, yolo_label, (x, y, radius)
    
    def process_image(self, image_path, verbose=True):
        """
        处理单张图片：随机决定是否添加圆点，保留原有标注
        """
        image_path = Path(image_path)
        image_stem = image_path.stem
        
        # 读取已有的标注
        existing_labels, parsed_labels = self.read_existing_labels(image_stem)
        
        # 随机决定是否为负样本
        is_negative = random.random() < self.negative_ratio
        
        # 如果是正样本，决定是否使用重叠模式
        is_overlap = False
        if not is_negative and parsed_labels:
            is_overlap = random.random() < self.overlap_ratio
        
        # 生成图像和标注
        img, new_label, dot_info = self.generate_dot(
            image_path, parsed_labels, is_negative, is_overlap
        )
        
        # 保存处理后的图像
        output_img_path = self.output_image_dir / image_path.name
        cv2.imwrite(str(output_img_path), img)
        
        # 保存标注文件
        label_path = self.output_label_dir / f"{image_stem}.txt"
        with open(label_path, 'w', encoding='utf-8') as f:
            # 写入原有的所有标注
            for label in existing_labels:
                f.write(label + '\n')
            # 如果是正样本，追加新的圆点标注
            if not is_negative and new_label is not None:
                f.write(new_label + '\n')
            # 如果是负样本，不追加任何新标注
        
        # 更新统计
        self.stats['total'] += 1
        if is_negative:
            self.stats['negative'] += 1
        else:
            self.stats['positive'] += 1
            if is_overlap:
                self.stats['overlap'] += 1
            else:
                self.stats['random'] += 1
        
        if verbose:
            if is_negative:
                print(f"  ⚪ {image_path.name}: 负样本（不添加圆点），原有{len(existing_labels)}个标注")
            elif is_overlap:
                x, y, r = dot_info
                print(f"  🔵 {image_path.name}: 圆心({x}, {y}), 半径{r}px, "
                      f"【重叠模式】落在其他目标中心，原有{len(existing_labels)}个标注")
            else:
                x, y, r = dot_info
                print(f"  ✅ {image_path.name}: 圆心({x}, {y}), 半径{r}px, "
                      f"【随机模式】原有{len(existing_labels)}个标注")
        
        return {
            'image_name': image_path.name,
            'output_image_path': str(output_img_path),
            'label_path': str(label_path),
            'original_labels_count': len(existing_labels),
            'is_negative': is_negative,
            'is_overlap': is_overlap,
            'new_label': new_label,
            'dot_info': dot_info
        }
    
    def process_folder(self, verbose=True):
        """批量处理文件夹中的所有图片"""
        results = []
        failed = []
        
        # 获取所有图片文件
        image_files = [f for f in self.image_dir.iterdir() 
                      if f.suffix.lower() in self.img_extensions]
        
        if not image_files:
            print(f"⚠️ 在 {self.image_dir} 中未找到图片文件")
            return results
        
        # 重置统计
        self.stats = {'total': 0, 'positive': 0, 'negative': 0, 
                      'overlap': 0, 'random': 0, 'failed': 0}
        
        print("=" * 70)
        print(f"📁 原始图像文件夹: {self.image_dir}")
        print(f"📁 原始标注文件夹: {self.label_dir}")
        print(f"📁 输出图像文件夹: {self.output_image_dir}")
        print(f"📁 输出标注文件夹: {self.output_label_dir}")
        print(f"🔵 圆点半径范围: {self.min_radius}-{self.max_radius}px")
        print(f"🟣 颜色: BGR{self.dot_color}")
        print(f"🏷️  新类别ID: {self.class_id}")
        print(f"📊 负样本比例: {self.negative_ratio * 100:.0f}%")
        print(f"📊 重叠比例（落在其他目标中心）: {self.overlap_ratio * 100:.0f}%")
        print(f"📊 共找到 {len(image_files)} 张图片")
        print("-" * 70)
        
        for idx, img_path in enumerate(image_files, 1):
            print(f"[{idx}/{len(image_files)}] 处理中...", end=" ")
            try:
                result = self.process_image(img_path, verbose=False)
                results.append(result)
                if verbose:
                    if result['is_negative']:
                        print(f"⚪ {result['image_name']}: 负样本，原有{result['original_labels_count']}个标注")
                    elif result['is_overlap']:
                        x, y, r = result['dot_info']
                        print(f"🔵 {result['image_name']}: 圆心({x}, {y}), 半径{r}px, "
                              f"【重叠】原有{result['original_labels_count']}个标注")
                    else:
                        x, y, r = result['dot_info']
                        print(f"✅ {result['image_name']}: 圆心({x}, {y}), 半径{r}px, "
                              f"【随机】原有{result['original_labels_count']}个标注")
            except Exception as e:
                print(f"❌ 失败: {e}")
                failed.append(img_path.name)
                self.stats['failed'] += 1
                continue
        
        print("-" * 70)
        print(f"✅ 处理完成!")
        print(f"   - 总处理: {self.stats['total']} 张")
        print(f"   - 正样本（有圆点）: {self.stats['positive']} 张 "
              f"({self.stats['positive']/max(1,self.stats['total'])*100:.1f}%)")
        print(f"     - 重叠模式（在目标中心）: {self.stats['overlap']} 张 "
              f"({self.stats['overlap']/max(1,self.stats['positive'])*100:.1f}%)")
        print(f"     - 随机模式: {self.stats['random']} 张 "
              f"({self.stats['random']/max(1,self.stats['positive'])*100:.1f}%)")
        print(f"   - 负样本（无圆点）: {self.stats['negative']} 张 "
              f"({self.stats['negative']/max(1,self.stats['total'])*100:.1f}%)")
        if failed:
            print(f"   - 失败: {len(failed)} 张")
            print(f"   - 失败列表: {', '.join(failed)}")
        print(f"   - 输出图像: {self.output_image_dir}")
        print(f"   - 输出标注: {self.output_label_dir}")
        print("=" * 70)
        
        return results


def main():
    """主函数"""
    
    # ========== 配置参数 ==========
    IMAGE_DIR = "../data/JInghao/images"
    LABEL_DIR = "../data/JInghao_labels"
    
    OUTPUT_IMAGE_DIR = "../data/JInghao_new/images"
    OUTPUT_LABEL_DIR = "../data/JInghao_new/labels"
    
    # 圆点参数
    MIN_RADIUS = 1                # 最小半径(像素)
    MAX_RADIUS = 10               # 最大半径(像素)
    CLASS_ID = 6                  # 新类别的ID
    DOT_COLOR = (0, 0, 255)       # 红色 (BGR)
    
    # 样本比例
    NEGATIVE_RATIO = 0.1          # 10%的图片不添加圆点
    OVERLAP_RATIO = 0.5           # 40%的圆点落在其他目标中心
    
    # ========== 执行处理 ==========
    generator = DotGenerator(
        image_dir=IMAGE_DIR,
        label_dir=LABEL_DIR,
        output_image_dir=OUTPUT_IMAGE_DIR,
        output_label_dir=OUTPUT_LABEL_DIR,
        min_radius=MIN_RADIUS,
        max_radius=MAX_RADIUS,
        dot_color=DOT_COLOR,
        class_id=CLASS_ID,
        negative_ratio=NEGATIVE_RATIO,
        overlap_ratio=OVERLAP_RATIO      # 新增参数
    )
    
    results = generator.process_folder(verbose=True)
    
    # ========== 生成报告 ==========
    if results:
        print("\n" + "=" * 70)
        print("📝 配置总结:")
        print("-" * 70)
        print(f"   负样本比例: {NEGATIVE_RATIO * 100:.0f}%")
        print(f"   重叠比例: {OVERLAP_RATIO * 100:.0f}%")
        print(f"   圆点半径: {MIN_RADIUS}-{MAX_RADIUS}px")
        print(f"   类别ID: {CLASS_ID}")
        print("-" * 70)
        print("📝 请更新你的 data.yaml 文件:")
        print("-" * 70)
        print(f"train: {OUTPUT_IMAGE_DIR}")
        print(f"val: {OUTPUT_IMAGE_DIR}  # 如果验证集也需要")
        print(f"\nnc: 原有类别数 + 1  # 新增圆点类别")
        print(f"names:")
        print(f"  # ... 原有的类别名称 ...")
        print(f"  {CLASS_ID}: 'purple_dot'")
        print("=" * 70)


if __name__ == "__main__":
    main()