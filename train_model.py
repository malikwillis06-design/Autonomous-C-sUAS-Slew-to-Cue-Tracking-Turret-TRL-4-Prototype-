from ultralytics import YOLO

def train_turret_brain():
    # 1. Start with the pretrained YOLO26 Nano (the base "eyes")
    # This model is lightweight and perfect for real-time turret tracking
    model = YOLO('yolo26n.pt') 
    
    # 2. Train on your custom dataset
    # Make sure 'data.yaml' is in the same folder as this script
    # This will use your NVIDIA GPU automatically via CUDA
    model.train(
        data='data.yaml', 
        epochs=50,          # Adjust this based on how long you want to train
        imgsz=640,          # Standard input size for high accuracy
        device=0,           # 0 indicates your primary NVIDIA GPU
        batch=16,           # Increase this if you have high VRAM, decrease if you get OOM errors
        name='turret_v1'    # This creates a folder in 'runs/detect/' with your results
    )

    print("Training Complete. Your custom weights are in runs/detect/turret_v1/weights/best.pt")

if __name__ == '__main__':
    train_turret_brain()