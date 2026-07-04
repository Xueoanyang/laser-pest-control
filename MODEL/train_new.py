# train.py - 增加针对夜间和运动模糊的增强
from ultralytics import YOLO
#import albumentations as A  # 需要先安装：pip install albumentations

def main():
    model = YOLO("yolo26n.pt")

    # 定义自定义的 Albumentations 增强管道
    # 这些增强会与 YOLO 自带的增强叠加使用
    # custom_augmentations = [
    #     # 模拟夜间效果
    #     A.RandomBrightnessContrast(brightness_limit=(-0.3, 0.1), contrast_limit=(-0.2, 0.3), p=0.5),
    #     A.GaussNoise(var_limit=(10.0, 50.0), p=0.3),
        
    #     # 模拟运动模糊
    #     A.MotionBlur(blur_limit=7, p=0.3),  # blur_limit 控制模糊程度
    #     A.Blur(blur_limit=3, p=0.2),
    # ]

    results = model.train(
        data="./data_Duong.yaml",
        resume=True,       # 恢复训练状态
        epochs=200,
        imgsz=640,
        batch=32,
        device=0,
        workers=16,
        name="yolo26_Duong_more2",
        exist_ok=True,
        
        # ========== 数据增强（保持不变） ==========
        scale=0.8,
        mosaic=1.0,
        mixup=0.3,
        degrees=15.0,
        translate=0.15,
        shear=8.0,
        perspective=0.0005,
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        fliplr=0.5,
        flipud=0.0,
        erasing=0.3,
        dropout=0.1,
        
        # ========== 集成 Albumentations 增强 ==========
        # 通过 augmentations 参数传入自定义管道
        #augmentations=custom_augmentations,  
        
        # 在最后 10 个 epoch 关闭 mosaic
        close_mosaic=10,
        
        # 其他优化参数
        optimizer='auto',
        lr0=0.01,
        lrf=0.01,
        momentum=0.937,
        weight_decay=0.0005,
        warmup_epochs=3.0,
        warmup_momentum=0.8,
        warmup_bias_lr=0.1,
        cache=False,
        amp=True,
        nbs=64,
        patience=30,
        save_period=10,
        plots=True,
        verbose=True,
    )
    
    print("训练完成！")

if __name__ == "__main__":
    main()