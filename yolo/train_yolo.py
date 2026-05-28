from ultralytics import YOLO

def train():
    # Carrega o modelo YOLO11 Nano (mais eficiente e recente que o V8 Nano)
    # Ideal para poupar processamento no Raspberry Pi 4.
    model = YOLO('yolo11n.pt')

    # 1. Treinamento
    # Baixamos imgsz para 320px: corta o processamento para 1/4 face aos 640px,
    # mantendo precisão suficiente para objetos grandes como rampa e caixas.
    results = model.train(
        data='data_local.yaml',
        epochs=100,
        imgsz=320,
        device=0,           # Sua RTX 4060 para treinar rápido
        project='speedy_vision',
        name='yolo11n_speedy_rpi'
    )

    print("\n--- Treinamento Concluído ---")

    # 2. Exportação para NCNN (O formato vital para o RPi 4)
    print("Exportando para NCNN (Alta performance ARM/RPi)...")
    model.export(format='ncnn', imgsz=320)

    print("\n--- Processo Finalizado ---")
    print("Os modelos otimizados estão na pasta 'yolo/speedy_vision/yolo11n_speedy_rpi/weights/'")

if __name__ == '__main__':
    train()
