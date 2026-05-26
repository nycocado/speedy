from ultralytics import YOLO

def train():
    # Carrega o modelo Nano
    model = YOLO('yolov8n.pt')

    # 1. Treinamento
    results = model.train(
        data='data_local.yaml',
        epochs=100,
        imgsz=320,          # Redução de tamanho = Mais FPS no RPi4
        device=0,           # Sua RTX 4060
        project='speedy_vision',
        name='yolov8n_speedy_final'
    )

    print("\n--- Treinamento Concluído ---")

    # 2. Exportação para NCNN (O formato mais rápido para o processador do RPi4)
    print("Exportando para NCNN (Máxima Performance)...")
    model.export(format='ncnn', imgsz=320)

    # 3. Exportação para TFLite (Onde o int8 é suportado)
    print("Exportando para TFLite INT8 (8-bits)...")
    model.export(format='tflite', imgsz=320, int8=True)

    print("\n--- Processo Finalizado ---")
    print("Os modelos otimizados estão na pasta 'yolo/speedy_vision/yolov8n_speedy_final/weights/'")

if __name__ == '__main__':
    train()
